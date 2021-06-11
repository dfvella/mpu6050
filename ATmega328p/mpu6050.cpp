#include "mpu6050.h"

// construct MPU6050 object
// OPTIONAL: provide a pin number to enable status led
MPU6050::MPU6050() : status_led(0)
{
    orientation.w = 0.70710;
    orientation.x = 0.70710;
    orientation.y = 0.00001;
    orientation.z = 0.00001;
}

// construct MPU6050 object
// OPTIONAL: provide a pin number to enable status led
MPU6050::MPU6050(int8_t pin) : status_led(pin) 
{
    orientation.w = 0.70710;
    orientation.x = 0.70710;
    orientation.y = 0.00001;
    orientation.z = 0.00001;
}

// Waits for rest then zeros gyro with reference to gravity
void MPU6050::begin() 
{
    if (status_led)
        pinMode(status_led, OUTPUT);

    config_sensors();

    #ifdef WAIT_FOR_REST
    wait_for_rest();
    #endif // WAIT_FOR_REST

    float calibration_data[7];
    avg_sample(calibration_data, GYRO_CALIBRATION_READINGS);

    x_zero = calibration_data[GYROX];
    y_zero = calibration_data[GYROY];
    z_zero = calibration_data[GYROZ];

    float x_accel = calibration_data[ACCELX] - ACCELX_LEVEL_READING;
    float y_accel = calibration_data[ACCELY] - ACCELY_LEVEL_READING;
    float z_accel = calibration_data[ACCELZ] - ACCELZ_LEVEL_READING;

    Vector net_accel = { x_accel, y_accel, z_accel };

    float x_angle_accel = asin(y_accel / norm(net_accel)) * (1 / RADIANS_PER_DEGREE);
    float y_angle_accel = asin(x_accel / norm(net_accel)) * (1 / RADIANS_PER_DEGREE) * -1;

    #ifdef ENABLE_GRAVITY_REFERENCED_ZERO

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

// Immediately begins sampling accelerometer data and prints the average
// of these readings to the serial monitor. Mpu6050 should be placed
// on a level surface.
// NOTE: Serial.begin() must be called before using this function
void MPU6050::calibrate_accel()
{
    if (status_led)
        pinMode(status_led, OUTPUT);

    config_sensors();

    Serial.println("Calibrating Accelerometer...");

    float calibration_data[7];
    avg_sample(calibration_data, ACCEL_CALIBRATION_READINGS);

    Serial.print(" x: ");
    Serial.print(calibration_data[ACCELX]);
    Serial.print(" y: ");
    Serial.print(calibration_data[ACCELY]);
    Serial.print(" z: ");
    Serial.print(calibration_data[ACCELZ] - TICKS_PER_G);
    Serial.println();
}

// Computes the IMU's orientation
void MPU6050::run() 
{
    fetch_data();

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

        Vector w;
        w.x = (data[GYROX] - x_zero) * TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        w.y = (data[GYROY] - y_zero) * TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
        w.z = (data[GYROZ] - z_zero) * TICKS_PER_DEGREE * RADIANS_PER_DEGREE;

        float w_norm = norm(w);

        Quaternion rotation;
        rotation.w = cos((t_delta * w_norm) / 2);
        rotation.x = (sin((t_delta * w_norm) / 2) * w.x) / w_norm;
        rotation.y = (sin((t_delta * w_norm) / 2) * w.y) / w_norm;
        rotation.z = (sin((t_delta * w_norm) / 2) * w.z) / w_norm;

        orientation = product(orientation, rotation);

        x_angle = atan2(2 * orientation.x * orientation.w - 2 * orientation.y * orientation.z, 
                1 - 2 * orientation.x * orientation.x - 2 * orientation.z * orientation.z);

        y_angle = asin(2 * orientation.x * orientation.y + 2 * orientation.z * orientation.w);

        z_angle = atan2(2 * orientation.y * orientation.w - 2 * orientation.x * orientation.z, 
                1 - 2 * orientation.y * orientation.y - 2 * orientation.z * orientation.z);

        x_angle /= RADIANS_PER_DEGREE;
        y_angle /= RADIANS_PER_DEGREE;
        z_angle /= RADIANS_PER_DEGREE;

        if (x_angle < -90) 
            x_angle += 270;
        else 
            x_angle -= 90;

        // inevert axes if necessary 
        #ifdef INVERT_ROLL_AXIS
        x_angle *= -1;
        #endif    
        #ifdef INVERT_PITCH_AXIS
        y_angle *= -1;
        #endif
        #ifdef INVERT_YAW_AXIS
        z_angle *= -1;
        #endif
    }
}

// Returns the roll angle of the aircraft in degrees
//  -180 < roll < 180
float MPU6050::roll() 
{
    return x_angle;
}

// Returns the pitch angle of the aircraft in degrees
//  -90 < pitch < 90
float MPU6050::pitch() 
{
    return y_angle;
}

// Returns the yaw angle of the aircraft in degrees
//  -180 < yaw < 180
float MPU6050::yaw() 
{
    return z_angle;
}

// Returns raw accelerometer, temperature, and gyroscope data
int32_t MPU6050::get_raw(uint8_t val)
{
    if (0 <= val && val < 7)
        return data[val];
    else
        return -1;
}

// set mpu6050 configuration registers
void MPU6050::config_sensors()
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

// fill data array with most recent sensor measurements
void MPU6050::fetch_data()
{
    Wire.beginTransmission(MPU6050_I2C_ADDRESS);
    Wire.write(MPU6050_ACCEL_XOUT_H_REGISTER);
    Wire.endTransmission();

    Wire.requestFrom(MPU6050_I2C_ADDRESS, 14);
    while (Wire.available() < 14);

    for (int32_t *ptr = data; ptr < data + 7; ++ptr)
    *ptr = Wire.read() << 8 | Wire.read();
}

// fill buffer with the average of n sensor readings
void MPU6050::avg_sample(float* buffer, uint16_t n)
{
    float x_gyro = 0;
    float y_gyro = 0;
    float z_gyro = 0;

    float x_accel = 0;
    float y_accel = 0;
    float z_accel = 0;

    unsigned long timer = micros();

    uint16_t count = 0;

    bool led_state = false;

    for (uint16_t i = 0; i < n; ++i) 
    {
        while (micros() - timer < 4000);
        timer = micros();

        fetch_data();

        x_gyro += data[GYROX] / (float)n;
        y_gyro += data[GYROY] / (float)n;
        z_gyro += data[GYROZ] / (float)n;

        x_accel += data[ACCELX] / (float)n;
        y_accel += data[ACCELY] / (float)n;
        z_accel += data[ACCELZ] / (float)n;

        if (status_led) 
        {
            if (++count % 25 == 0)
                led_state = !led_state;
            digitalWrite(status_led, led_state); 
        }
    }

    buffer[GYROX] = x_gyro;
    buffer[GYROY] = y_gyro;
    buffer[GYROZ] = z_gyro;

    buffer[ACCELX] = x_accel;
    buffer[ACCELY] = y_accel;
    buffer[ACCELZ] = z_accel;
}

// blocks until accelerometer
void MPU6050::wait_for_rest()
{
    bool led_state = false;

    uint16_t count = 0;
    uint16_t rest = 0;

    int16_t x_prev, y_prev, z_prev;

    unsigned long timer = 0;

    while (rest < PRE_CALIBRATION_REST_TIMER) 
    {
        while (micros() - timer < 4000);
        timer = micros();

        fetch_data();

        uint16_t x_diff = abs(data[ACCELX] - x_prev);
        uint16_t y_diff = abs(data[ACCELY] - y_prev);
        uint16_t z_diff = abs(data[ACCELZ] - z_prev);

        x_prev = data[ACCELX];
        y_prev = data[ACCELY];
        z_prev = data[ACCELZ];

        if (x_diff < MIN_ACCEL_DIFF && y_diff < MIN_ACCEL_DIFF && z_diff < MIN_ACCEL_DIFF)
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
}

// computes then returns the product of two quaternions
MPU6050::Quaternion MPU6050::product(const Quaternion &p, const Quaternion &q) 
{
    Quaternion result;

    result.w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z;
    result.x = p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y;
    result.y = p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x;
    result.z = p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w;

    // adjust dimensions to ensure the norm is 1 for noise reduction
    float l = norm(result);
    result.w /= l;
    result.x /= l;
    result.y /= l;
    result.x /= l;

    return result;
}

// Computes the magnitude of quaternion q
float MPU6050::norm(const Quaternion &q) 
{
    return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// Computes the magnitude of vector v
float MPU6050::norm(const Vector &v) 
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
