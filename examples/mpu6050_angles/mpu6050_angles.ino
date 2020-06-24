/*
 * Example sketch showcasing use of the mpu6050 Arduino library to
 * calculate roll, pitch, and yaw angles.
 * 
 * Written by David Vella, June 2020
 */

#include <mpu6050.h>

const int led_pin = 13;

// Optional: pass an led pin number into the constructor to enable
// the led indicator
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
        // call run() within loop() 50-250 Hz for best results.
        // This method computes the change in orientation between calls
        // NOTE: this function is quite expensive. Take about 2 
        // milliseconds to execute on ATmega328p
        imu.run();

        delay(4);
    }

    // roll() returns an angle in degrees (-180 < roll < 180)
    Serial.print(imu.roll());
    Serial.print(' ');

    // pitch() returns an angle in degrees (-90 < pitch < 90)
    Serial.print(imu.pitch());
    Serial.print(' ');

    // yaw() returns and angle in degrees (-180 < yaw < 180)
    Serial.println(imu.yaw());
}