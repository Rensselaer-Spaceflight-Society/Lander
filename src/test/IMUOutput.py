import time
import board
import adafruit_mpu6050

try: 
    i2c = board.I2C()  # uses board.SCL and board.SDA
    mpu = adafruit_mpu6050.MPU6050(i2c)
    mpu.accelerometer_range = adafruit_mpu6050.Range.RANGE_2_G
    mpu.gyro_range = adafruit_mpu6050.GyroRange.RANGE_250_DPS
except:
    print("No I2C connection")

print()

while True:
    try:
        print("Acceleration X: {:.2f}".format(mpu.acceleration[0]), end=", ")
        print("Y: {:.2f}".format(mpu.acceleration[1]), end=", ")
        print("Z: {:.2f} m/s^2".format(mpu.acceleration[2]))
        # print("(%.2f, %.2f, %.2f " % (mpu.acceleration), end=", ")
        print("Gyro X: {:.2f}".format(mpu.gyro[0]), end=", ")
        print("Y: {:.2f}".format(mpu.gyro[1]), end=", ")
        print("Z: {:.2f} rad/s".format(mpu.gyro[2]))
        # print("%.2f, %.2f, %.2f)" % (mpu.gyro))
        print()
        time.sleep(0.5)
    except:
        print("Lost connection")

        try: 
            i2c = board.I2C()  # uses board.SCL and board.SDA
            mpu = adafruit_mpu6050.MPU6050(i2c)
            mpu.accelerometer_range = adafruit_mpu6050.Range.RANGE_2_G
            mpu.gyro_range = adafruit_mpu6050.GyroRange.RANGE_250_DPS
        except:
            print("No I2C connection")