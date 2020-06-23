/*
 * Example sketch showcasing use of the mpu6050 Arduino library to
 * calculate roll, pitch, and yaw angles.
 * 
 * Written by David Vella, June 2020
 */

#include <mpu6050.h>

const int led_pin = 13;

Imu imu(led_pin);

void setup()
{
    Serial.begin(9600);

    pinMode(led_pin, OUTPUT);

    // call calibrate() within setup()
    imu.calibrate();
}

void loop()
{
    for (int i = 0; i < 250; ++i)
    {
        // call run() within loop() 50-250 Hz for best results
        imu.run();

        delay(4);
    }

    Serial.print(imu.roll());
    Serial.print(' ');
    Serial.print(imu.pitch());
    Serial.print(' ');
    Serial.println(imu.yaw());
}