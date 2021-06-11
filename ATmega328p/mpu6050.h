#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <Arduino.h>
#include <Wire.h>

// ********** LED INDICATOR CODES **********
// slow blink - waiting for aircraft to be stationary
// fast blink - calibrating (do not move)

// ********** ACCELEROMETER CALIBRATION **********
#define ACCELX_LEVEL_READING 79.40
#define ACCELY_LEVEL_READING -103.94
#define ACCELZ_LEVEL_READING 1225.42

// ********** INVERT AXES **********
#define INVERT_ROLL_AXIS
//#define INVERT_PITCH_AXIS
//#define INVERT_YAW_AXIS

// ********** IMU CALIBRATION ROUTINE SETTINGS **********
#define PRE_CALIBRATION_REST_TIMER 1500
#define MIN_ACCEL_DIFF 200
#define GYRO_CALIBRATION_READINGS 1000
#define ACCEL_CALIBRATION_READINGS 5000
// When enabled, the sensor does not have to be level during calibration.
// Instead, the net acceleration vector will be used to determine initial
// orientation.
#define ENABLE_GRAVITY_REFERENCED_ZERO
// When enabled, acceleration data will be used to wait for rest before
// starting the calibration routine in calibrate().
#define WAIT_FOR_REST


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


class MPU6050 
{
public:
    // construct MPU6050 object
    // OPTIONAL: provide a pin number to enable status led
    MPU6050();

    // construct MPU6050 object
    // OPTIONAL: provide a pin number to enable status led
    MPU6050(int8_t pin);

    // Waits for rest then zeros gyro with reference to gravity
    void begin();

    // Immediately begins sampling accelerometer data and prints the average
    // of these readings to the serial monitor. Mpu6050 should be placed
    // on a level surface.
    // NOTE: Serial.begin() must be called before using this function
    void calibrate_accel();

    // Updates the IMU's orientation
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

    // Returns raw accelerometer, temperature, and gyroscope data
    int32_t get_raw(uint8_t val);

    static constexpr uint8_t ACCELX = 0; // Index for raw data array
    static constexpr uint8_t ACCELY = 1; // Index for raw data array
    static constexpr uint8_t ACCELZ = 2; // Index for raw data array
    static constexpr uint8_t TEMP = 3;   // Index for raw data array
    static constexpr uint8_t GYROX = 4;  // Index for raw data array
    static constexpr uint8_t GYROY = 5;  // Index for raw data array
    static constexpr uint8_t GYROZ = 6;  // Index for raw data array

private:
    static constexpr float RADIANS_PER_DEGREE = 0.01745329;
    static constexpr float TICKS_PER_DEGREE = 0.0152672;
    static constexpr float TICKS_PER_G = 4096.0;

    // set mpu6050 configuration registers
    void config_sensors();

    // fill data array with most recent sensor measurements
    void fetch_data();

    // fill buffer with the average of n sensor readings
    void avg_sample(float* buffer, uint16_t n);

    // blocks until accelerometer
    void wait_for_rest();

    struct Quaternion 
    {
        float w, x, y, z;
    };

    // returns the product of two quaternions
    Quaternion product(const Quaternion &p, const Quaternion &q);

    // returns the norm/length of a quaternion
    float norm(const Quaternion &q);

    struct Vector 
    {
        float x, y, z;
    };

    // returns the norm/length of a vector
    float norm(const Vector &v);

    Quaternion orientation;

    float x_angle, y_angle, z_angle;
    float x_zero, y_zero, z_zero;

    unsigned long timer;

    int32_t data[7];

    uint8_t status_led;
};

#endif // __MPU6050_H__
