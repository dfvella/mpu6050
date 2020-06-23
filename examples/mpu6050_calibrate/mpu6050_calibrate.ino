/*
 * Sketch for calibraing the mpu6050 imu. Prints average accelerometer
 * readings to the serial monitor
 * 
 * Written by David Vella, June 2020
 */

// ***** CALIBRATION INTRUSCTIONS *****
// - Place mpu6050 on a level surface
// - Upload the sketch and open serial monitor
// - After sketch completes, enter values that appear in the serial
//   monitor into the macro definitions at the top of imu.h

#include <mpu6050.h>

#define CALIBRATION_READINGS 2500

Mpu6050 mpu;

void setup()
{
    Serial.begin(9600);
    Serial.println("Calibrating accelerometer...");

    mpu.begin();

    float x = 0, y = 0, z = 0;

    for (int i = 0; i < CALIBRATION_READINGS; ++i)
    {
        mpu.fetch();

        x += mpu.get(Mpu6050::ACCELX) / (float)CALIBRATION_READINGS;
        y += mpu.get(Mpu6050::ACCELY) / (float)CALIBRATION_READINGS;
        z += mpu.get(Mpu6050::ACCELZ) / (float)CALIBRATION_READINGS;

        if (i % (CALIBRATION_READINGS / 10) == 0)
            Serial.print('#');

        delay(4);
    }

    Serial.println(" Finished");

    Serial.println("----------------");
    Serial.print("x: ");
    Serial.println(x);
    Serial.print("y: ");
    Serial.println(y);
    Serial.print("z: ");
    Serial.println(z - Mpu6050::TICKS_PER_G);
    Serial.println("----------------");
}

void loop() { }