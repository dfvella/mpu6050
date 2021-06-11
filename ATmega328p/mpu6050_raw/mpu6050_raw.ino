/*
 * Example sketch showcasing use of the mpu6050 library
 */

#include "mpu6050.h"

#define PRINT_RAW
#define PRINT_ANGLES
#define CALIBRATE

// create an instance of Mpu6050
MPU6050 mpu6050(LED_BUILTIN);

void setup()
{
    Serial.begin(9600);

    // call begin() within setup()
    mpu6050.begin();
}

void loop()
{
    // call fetch() within loop().
    mpu6050.run();

    int data[6];

    // call get() to retrieve raw data
    data[0] = mpu6050.get_raw(MPU6050::ACCELX);
    data[1] = mpu6050.get_raw(MPU6050::ACCELY);
    data[2] = mpu6050.get_raw(MPU6050::ACCELZ);
    data[3] = mpu6050.get_raw(MPU6050::GYROX);
    data[4] = mpu6050.get_raw(MPU6050::GYROY);
    data[5] = mpu6050.get_raw(MPU6050::GYROZ);

    for (int i = 0; i < 6; ++i)
    {
        Serial.print(data[i]);
        Serial.print(' ');
    }
    Serial.println();

    delay(100);
}