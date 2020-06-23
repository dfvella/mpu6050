/*
 * Class for calculating angles from raw gyro and accelerometer
 * data from the MPU6050 sensor
 * 
 * Written by David Vella, March 2020
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

#include "mpu6050.h"

// ********** LED INDICATOR CODES **********
// slow blink - waiting for aircraft to be stationary
// fast blink - calibrating (do not move)

// ********** ACCELEROMETER CALIBRATION **********
#define ACCELX_LEVEL_READING 0 
#define ACCELY_LEVEL_READING 0 // <--- enter values from calibarion sketch
#define ACCELZ_LEVEL_READING 0

// ********** INVERT AXES **********
//#define INVERT_ROLL_AXIS
//#define INVERT_PITCH_AXIS
//#define INVERT_YAW_AXIS

// ********** IMU CALIBRATION ROUTINE SETTINGS **********
#define GYRO_CALIBRATION_READINGS 1000
//#define WAIT_FOR_REST
#define PRE_CALIBRATION_REST_TIMER 1500
#define MIN_ACCEL_DIFF 200

// sets initial orientation using net acceleration vector
//#define ENABLE_GRAVITY_REFERENCED_ZERO


class Imu 
{
    public:
        Imu();

        // OPTIONAL: pin - status led pin number
        Imu(int8_t pin);

        void calibrate();

        // Computes the IMU's orientation
        void run();

        // Returns the roll angle of the aircraft in degrees
        //  -180 < roll < 180
        float roll();

        // Returns the pitch angle of the aircraft in degrees
        //  -90 < pitch < 90
        float pitch();

        // Returns the yaw angle of the aircraft in degrees
        //  -180 < yaw < 180
        float yaw();

    private:
        static constexpr float RADIANS_PER_DEGREE = 0.01745329;

        struct Quaternion 
        {
            float w, x, y, z;
        };

        // returns the product of two quaternions
        Quaternion product(const Quaternion &p, const Quaternion &q);

        // returns the magnitude of Quaternion q
        float norm(const Quaternion &q);

        struct Vector 
        {
            float x, y, z;
        };

        // returns the magnitude of vector v
        float norm(const Vector &v);

        Quaternion orientation;

        // stores gyro raw output when at rest
        float x_zero, y_zero, z_zero;

        // stores time when orientation was last updated
        unsigned long timer;

        Mpu6050 mpu;

        uint8_t status_led;
};

#endif // IMU_H
