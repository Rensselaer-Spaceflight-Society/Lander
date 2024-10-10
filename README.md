# Project Overview

This project demonstrates the integration of a PCA9685 servo driver, MPU6050 (Gyroscope and Accelerometer), and basic video capture using OpenCV. It combines sensor data from the MPU6050 to control servo movements, while the OpenCV part captures live video from a connected camera.

## Requirements

- **Raspberry Pi** (or any platform supporting I2C and GPIO)
- **MPU6050** (Gyroscope and Accelerometer sensor)
- **PCA9685** (16-channel, 12-bit PWM servo driver)
- **Servo Motor**
- **OpenCV** (for camera video capture)
- **Adafruit CircuitPython Servokit** (for controlling the PCA9685)

## Components Breakdown

### 1. PCA9685 Servo Driver

The PCA9685 is a 16-channel PWM driver used to control multiple servos. In this project, it controls a servo motor, adjusting its angle based on gyroscope data from the MPU6050. The pulse width range and angle of the servo are configured using:

- `pca = ServoKit(channels=16)`
- `pca.servo[0].set_pulse_width_range(MIN_IMP[0], MAX_IMP[0])`
- The servo angle is adjusted dynamically based on gyroscope input.

### 2. MPU6050 (Gyroscope and Accelerometer)

The MPU6050 is an I2C-based 6-axis motion tracking sensor that provides accelerometer and gyroscope data. The MPU is initialized and configured using specific register addresses, and raw data is read via I2C. The gyroscope and accelerometer values are calculated and processed for further use in controlling the servo motor.

- Registers like `PWR_MGMT_1`, `GYRO_CONFIG`, and `ACCEL_XOUT_H` are used to configure the sensor and fetch raw data.
- The raw data is converted into meaningful gyroscope values, which are then used to determine the servo's angle.

### 3. OpenCV for Camera Capture

A simple implementation of OpenCV is used to capture real-time video from a camera:

- `cv2.VideoCapture(0)` initializes the camera.
- The live video stream is displayed in a window, and pressing 'q' on the keyboard will terminate the video capture loop.

### Code Behavior

- **MPU6050 sensor**: Continuously reads gyroscope and accelerometer data.
- **Servo Control**: The servo angle is modified based on gyroscope readings (`Gx > 0.15` results in a servo angle of 90 degrees).
- **Camera**: Captures and displays live video from the connected camera.

### Key Libraries Used

- **Adafruit CircuitPython ServoKit**: Used to control the PCA9685 servo driver.
- **smbus**: Provides I2C communication with the MPU6050 sensor.
- **RPi.GPIO**: Controls GPIO pins for PWM, but it is not actively used in this setup (commented out).
- **OpenCV (cv2)**: Handles video capture and display.

### MPU6050 Register Map

The project uses several key MPU6050 registers, such as:
- `PWR_MGMT_1`: Power management.
- `ACCEL_XOUT_H`, `ACCEL_YOUT_H`, `ACCEL_ZOUT_H`: Accelerometer readings.
- `GYRO_XOUT_H`, `GYRO_YOUT_H`, `GYRO_ZOUT_H`: Gyroscope readings.

### Usage

1. Ensure the PCA9685 is connected to the Raspberry Pi via I2C.
2. Connect the MPU6050 sensor to the Raspberry Pi's I2C pins.
3. Connect the servo to one of the channels on the PCA9685.
4. Run the code. The servo motor's angle will be adjusted based on the gyroscope readings, and a live video feed will be displayed.

### Notes

- Ensure the I2C address of the MPU6050 is correctly set (`Device_Address = 0x68`).
- Install necessary dependencies, including `opencv-python`, `adafruit-circuitpython-servokit`, and `smbus` for I2C communication.
