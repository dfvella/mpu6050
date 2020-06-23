/*
 * Class for calculating angles from raw gyro and accelerometer
 * data from the MPU6050 sensor
 * 
 * Written by David Vella, March 2020
 */

#include "imu.h"

// OPTIONAL: pin - status led pin number
Imu::Imu(int8_t pin) : status_led(pin) 
{
    orientation.w = 0.70710;
    orientation.x = 0.70710;
    orientation.y = 0.00001;
    orientation.z = 0.00001;
}

Imu::Imu() : Imu(0) { }

void Imu::calibrate() 
{
    mpu.begin();

    bool led_state = false;

    uint16_t count = 0;

#ifdef WAIT_FOR_REST

    uint16_t rest = 0;

    int16_t x_prev, y_prev, z_prev;

    // Wait until the derivative of the acceleration along each axis is below 
    // the MIN_ACCEL_DIFF for the duration of the PRE_CALIBRATION_REST_TIMER
    timer = 0;
    while (rest < PRE_CALIBRATION_REST_TIMER) 
    {
        while (micros() - timer < 4000);
        timer = micros();

        mpu.fetch();

        uint16_t x_diff = abs(mpu.get(Mpu6050::ACCELX) - x_prev);
        uint16_t y_diff = abs(mpu.get(Mpu6050::ACCELY) - y_prev);
        uint16_t z_diff = abs(mpu.get(Mpu6050::ACCELZ) - z_prev);

        x_prev = mpu.get(Mpu6050::ACCELX);
        y_prev = mpu.get(Mpu6050::ACCELY);
        z_prev = mpu.get(Mpu6050::ACCELZ);

        if (x_diff < MIN_ACCEL_DIFF 
         && y_diff < MIN_ACCEL_DIFF 
         && z_diff < MIN_ACCEL_DIFF)
            ++rest;
        else 
            rest = 0;

        if (status_led)
        {
            if (++count % 200 == 0) 
                led_state = !led_state;
            
            digitalWrite(status_led, led_state);
        }
    }

#endif // WAIT_FOR_REST

    x_zero = 0;
    y_zero = 0;
    z_zero = 0;

    float accel_x = 0;
    float accel_z = 0;
    float accel_y = 0;

    // Sample the gyroscope and accelerometer the number of timer prescribed by
    // GYRO_CALIBRATION_READINGS. Average all these raw values.
    for (uint16_t i = 0; i < GYRO_CALIBRATION_READINGS; ++i) 
    {
        while (micros() - timer < 4000);
        timer = micros();

        mpu.fetch();

        x_zero += mpu.get(Mpu6050::GYROX);
        y_zero += mpu.get(Mpu6050::GYROY);
        z_zero += mpu.get(Mpu6050::GYROZ);

        accel_x += mpu.get(Mpu6050::ACCELX);
        accel_y += mpu.get(Mpu6050::ACCELY);
        accel_z += mpu.get(Mpu6050::ACCELZ);

        if (status_led) 
        {
            if (++count % 25 == 0) 
                led_state = !led_state;
            
            digitalWrite(status_led, led_state); 
        }
    }

    x_zero /= GYRO_CALIBRATION_READINGS;
    y_zero /= GYRO_CALIBRATION_READINGS;
    z_zero /= GYRO_CALIBRATION_READINGS;

    // sets initial orientation using net acceleration vector
#ifdef ENABLE_GRAVITY_REFERENCED_ZERO

    accel_x /= GYRO_CALIBRATION_READINGS;
    accel_y /= GYRO_CALIBRATION_READINGS;
    accel_z /= GYRO_CALIBRATION_READINGS;

    accel_x -= ACCELX_LEVEL_READING;
    accel_y -= ACCELY_LEVEL_READING;
    accel_z -= ACCELZ_LEVEL_READING;

    Vector net_accel = { 
        accel_x, 
        accel_y, 
        accel_z 
    };

    float x_angle_accel = asin(accel_y / norm(net_accel)) * (1 / RADIANS_PER_DEGREE);
    float y_angle_accel = asin(accel_x / norm(net_accel)) * (1 / RADIANS_PER_DEGREE) * -1;

    Quaternion initial_roll = {
        cos(x_angle_accel * RADIANS_PER_DEGREE * 0.5), 
        sin(x_angle_accel * RADIANS_PER_DEGREE * 0.5),
        0.0001,
        0.0001
    };

    orientation = product(orientation, initial_roll);

    Quaternion initial_pitch = {
        cos(y_angle_accel * RADIANS_PER_DEGREE * 0.5), 
        0.0001, 
        sin(y_angle_accel * RADIANS_PER_DEGREE * 0.5), 
        0.0001 
    };

    orientation = product(orientation, initial_pitch);

#endif // ENABLE_GRAVITY_REFERENCED_ZERO
}

// Computes the IMU's orientation
void Imu::run()
{
    mpu.fetch();

    static bool start = true;

    if (start) 
    {
        start = false;
        timer = micros();
    }
    else 
    {
        float t_delta = (micros() - timer) / 1000000.0; // t_delta units: seconds
        timer = micros();

        Vector w; // 3d axis of rotation

        w.x = (mpu.get(Mpu6050::GYROX) - x_zero) * Mpu6050::TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        w.y = (mpu.get(Mpu6050::GYROY) - y_zero) * Mpu6050::TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        w.z = (mpu.get(Mpu6050::GYROZ) - z_zero) * Mpu6050::TICKS_PER_DEGREE * RADIANS_PER_DEGREE;

        float w_norm = norm(w);

        if (w_norm == 0)
            w_norm = 0.0000001;

        Quaternion rotation;

        // integrate angular rate over the interval delta_t
        rotation.w = cos((t_delta * w_norm) / 2);
        rotation.x = (sin((t_delta * w_norm) / 2) * w.x) / w_norm;
        rotation.y = (sin((t_delta * w_norm) / 2) * w.y) / w_norm;
        rotation.z = (sin((t_delta * w_norm) / 2) * w.z) / w_norm;

        // apply rotation
        orientation = product(orientation, rotation);
    }
}

// Returns the roll angle of the aircraft in degrees
//  -180 < roll < 180
float Imu::roll() 
{
    float angle = atan2(
        2 * orientation.x * orientation.w - 2 * orientation.y * orientation.z, 
        1 - 2 * orientation.x * orientation.x - 2 * orientation.z * orientation.z
    );

    angle /= RADIANS_PER_DEGREE;

    if (angle < -90) 
        angle += 270;
    else 
        angle -= 90;

#ifdef INVERT_PITCH_AXIS
    angle *= -1;
#endif   

    return angle;
}

// Returns the pitch angle of the aircraft in degrees
//  -90 < pitch < 90
float Imu::pitch() 
{
    float angle = asin(2 * orientation.x * orientation.y + 2 * orientation.z * orientation.w);

    angle /= RADIANS_PER_DEGREE;

#ifdef INVERT_PITCH_AXIS
    angle *= -1;
#endif   

    return angle;
}


// Returns the yaw angle of the aircraft in degrees
//  -180 < yaw < 180
float Imu::yaw() 
{
    float angle = atan2(
        2 * orientation.y * orientation.w - 2 * orientation.x * orientation.z, 
        1 - 2 * orientation.y * orientation.y - 2 * orientation.z * orientation.z
    );

    angle /= RADIANS_PER_DEGREE;

#ifdef INVERT_YAW_AXIS
    angle *= -1;
#endif   

    return angle;
}

// returns the product of two quaternions p and q
Imu::Quaternion Imu::product(const Quaternion &p, const Quaternion &q) 
{
    Quaternion result;

    result.w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z;
    result.x = p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y;
    result.y = p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x;
    result.z = p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w;

    // normalize quaternion for noise reduction
    float l = norm(result);

    result.w /= l;
    result.x /= l;
    result.y /= l;
    result.x /= l;

    return result;
}

// returns the magnitude of Quaternion q
float Imu::norm(const Quaternion &q) 
{
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// returns the magnitude of Vector v
float Imu::norm(const Vector &v) 
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
