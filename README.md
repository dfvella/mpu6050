# About

An Arduino library that makes it easy to compute roll, pitch, and yaw angles from the mpu6050's raw gyroscope and accelerometer data. The implementation uses quaternions. Learn more [here](https://ltu.diva-portal.org/smash/get/diva2:1010947/FULLTEXT01.pdf). The library offers a simple interface. See [Examples](#Examples).

- Create an instance of the class [ex.```Imu imu;``` or ```Imu imu(led_pin);```]. Optional: pass in an pin number of an LED to enable the status indicator.
- ```calibrate()``` - call once in setup()
- ```run()``` - call in ```loop()``` between 50Hz and 250Hz (mpu6050 sampling rate). More frequent the better as this method computes the change in orientation between calls. Also note this function requires about 2 milliseconds to execute on the ATmega328p.
- ```roll()``` - returns roll angle in degrees (-180 < roll < 180)
- ```pitch()``` - returns pitch angle in degrees (-90 < pitch < 90)
- ```yaw()``` - returns yaw angle in degrees (-180 < yaw < 180)

By default, the imu reads 0 degrees at the angle at which the imu was initialized. The calibration process can be configured such that the accelerometer is used to compute initial roll and pitch angles relative to earth. Before enabling this feature the accelerometer should be calibrated using the example sketch. Enable this feature by un-commenting ```ENABLE_GRAVITY_REFERENCED_ZERO``` in ```imu.h```. Additionally, by default the imu will immediately begin the calibration routine upon a call to ```calibrate()```. Un-commenting ```WAIT_FOR_REST``` will delay the calibration routine until the MPU6050 is stationary.

# Installation

1. Download and extract the files. 
2. Rename the directory to ```mpu6050```. 
3. Make sure the files above are in the top level of ```mpu6050```. 
4. Move ```mpu6050``` to ```{your path}/Arduino/libraries```.
5. Restart Arduino IDE. ```Sketch > Include Library``` mpu6050 should be listed under contributed libraries

# Examples

[mpu6050_raw.ino](https://github.com/David-Vella/mpu6050/tree/master/examples/mpu6050_raw/mpu6050_raw.ino) - prints raw MPU6050 data to the serial monitor  
[mpu6050_raw.ino](https://github.com/David-Vella/mpu6050/tree/master/examples/mpu6050_angles/mpu6050_angles.ino) - prints roll, yaw, and pitch angles to the serial monitor  
[mpu6050_raw.ino](https://github.com/David-Vella/mpu6050/tree/master/examples/mpu6050_calibrate/mpu6050_calibrate.ino) - samples accelerometer for 10 seconds then prints average of raw data 

Written by David Vella, 2020