/*
 * Example sketch showcasing use of the mpu6050 Arduino library to
 * get raw sensor data.
 * 
 * Written by David Vella, June 2020
 */

#include <mpu6050.h>

// create an instance of Mpu6050
Mpu6050 mpu;

void setup()
{
    Serial.begin(9600);

    // call begin() within setup()
    mpu.begin();
}

void loop()
{
    // call fetch() within loop().
    mpu.fetch();

    int data[6];

    // call get() to retrieve raw data
    data[0] = mpu.get(Mpu6050::ACCELX);
    data[1] = mpu.get(Mpu6050::ACCELY);
    data[2] = mpu.get(Mpu6050::ACCELZ);
    data[3] = mpu.get(Mpu6050::GYROX);
    data[4] = mpu.get(Mpu6050::GYROY);
    data[5] = mpu.get(Mpu6050::GYROZ);

    for (int i = 0; i < 6; ++i)
    {
        Serial.print(data[i]);
        Serial.print(' ');
    }
    Serial.println();

    delay(1000);
}