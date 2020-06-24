# About

An Arduino library that makes it easy to compute roll, pitch, and yaw angles with the MPU6050 sensor. The library handles the I2C interface with the MPU6050 and uses quaternions to compute 3D orientation from the raw sensor data. Learn more about quaternion math for IMU's [here](https://ltu.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf). The library offers a simple interface. See [Examples](#Examples).

# Installation

1. Download and extract the files. 
2. Rename the directory to ```mpu6050```. 
3. Make sure the files above are in the top level of ```mpu6050```. 
4. Move ```mpu6050``` to ```{your path}/Arduino/libraries```.
5. Restart Arduino IDE. ```Sketch > Include Library``` mpu6050 should be listed under contributed libraries

# Hardware

See [wire_diagram.png](wire_diagram.png) for an example wire diagram with the MPU6050 connected with the Arduino Nano via I2C.

# Configuration

By default, the imu reads 0 degrees at the angle at which the imu was initialized. The calibration process can be configured such that the accelerometer is used to compute initial roll and pitch angles relative to earth. Before enabling this feature the accelerometer should be calibrated using the example sketch. Enable this feature by un-commenting `ENABLE_GRAVITY_REFERENCED_ZERO` in `imu.h`.

Additionally, by default the imu will immediately begin the calibration routine upon a call to `calibrate()`. Un-commenting `WAIT_FOR_REST` will delay the calibration routine until the MPU6050 is stationary.

# Examples

[mpu6050_raw.ino](https://github.com/David-Vella/mpu6050/tree/master/examples/mpu6050_raw/mpu6050_raw.ino) - prints raw MPU6050 data to the serial monitor  
[mpu6050_angles.ino](https://github.com/David-Vella/mpu6050/tree/master/examples/mpu6050_angles/mpu6050_angles.ino) - prints roll, yaw, and pitch angles to the serial monitor  
[mpu6050_calibrate.ino](https://github.com/David-Vella/mpu6050/tree/master/examples/mpu6050_calibrate/mpu6050_calibrate.ino) - samples accelerometer for 10 seconds then prints average of raw data. Use to calibrate accelerometer. 

Written by David Vella, 2020