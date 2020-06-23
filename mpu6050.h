/*
 * Class for interfacing with the MPU6050 sensor across I2C
 * 
 * Written by David Vella, March 2020
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

// ********** MPU6050 I2C REGISTER ADDRESSES **********
#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_POWER_MANAGEMENT_REGISTER 0x6B 
#define MPU6050_GYRO_CONFIG_REGISTER 0x1B
#define MPU6050_ACCELEROMETER_CONFIG_REGISTER 0x1C
#define MPU6050_ACCEL_XOUT_H_REGISTER 0x3B

// ********** MPU6050 POWER SETTINGS **********
#define MPU_6050_POWER_ON 0b00000000

// ********** MPU6050 GYROSCOPE ACCURACY SETTINGS **********
//#define GYRO_ACCURACY 0b00000000       // +/-  250 deg/sec
#define GYRO_ACCURACY 0b00001000       // +/-  500 deg/sec
//#define GYRO_ACCURACY 0b00010000       // +/- 1000 deg/sec
//#define GYRO_ACCURACY 0b00011000       // +/- 2000 deg/sec

// ********** MPU6050 ACCELEROMETER ACCURACY SETTINGS **********
//#define ACCELEROMETER_ACCURACY 0b00000000       // +/-  2 g's
//#define ACCELEROMETER_ACCURACY 0b00001000       // +/-  4 g's
#define ACCELEROMETER_ACCURACY 0b00010000       // +/-  8 g's
//#define ACCELEROMETER_ACCURACY 0b00011000       // +/- 16 g's


class Mpu6050 
{
    public:
        Mpu6050();

        // Configures the Mpu6050 sensor
        void begin();

        // Fills data array with the most recent MPU6050 measurements
        void fetch();

        // Returns the requested raw MPU6050 data
        // val - one of the following constant expressions:
        // ACCELX, ACCELY, ACCELZ, GYROX, GYROY, GYROZ
        int32_t get(uint8_t val);

        static constexpr uint8_t ACCELX = 0;
        static constexpr uint8_t ACCELY = 1;
        static constexpr uint8_t ACCELZ = 2;
        static constexpr uint8_t GYROX = 4;
        static constexpr uint8_t GYROY = 5;
        static constexpr uint8_t GYROZ = 6;

        static constexpr float TICKS_PER_DEGREE = 0.0152672;
        static constexpr float TICKS_PER_G = 4096.0;

    private:
        int32_t data[7];
};

#include "imu.h"

#endif