/*
 * Class for interfacing with the MPU6050 sensor across I2C
 * 
 * Written by David Vella, March 2020
 */

#include "mpu6050.h"


Mpu6050::Mpu6050() { }

// Configures the MPU6050 sensor
void Mpu6050::begin() 
{
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(MPU6050_POWER_MANAGEMENT_REGISTER);
    Wire.write(MPU_6050_POWER_ON);
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(MPU6050_ACCELEROMETER_CONFIG_REGISTER);
    Wire.write(ACCELEROMETER_ACCURACY);
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(MPU6050_GYRO_CONFIG_REGISTER);
    Wire.write(GYRO_ACCURACY);
    Wire.endTransmission();
}

// Returns requested raw MPU6050 data
void Mpu6050::fetch() 
{
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(MPU6050_ACCEL_XOUT_H_REGISTER);
    Wire.endTransmission();

    Wire.requestFrom(MPU6050_I2C_ADDRESS, 14);
    while (Wire.available() < 14);

    for (int32_t *ptr = data; ptr < data + 7; ++ptr)
        *ptr = Wire.read() << 8 | Wire.read();
}

// Returns the requested raw MPU6050 data
// val - one of the following constant expressions:
// ACCELX, ACCELY, ACCELZ, GYROX, GYROY, GYROZ
int32_t Mpu6050::get(uint8_t val) 
{
    return data[val];
}