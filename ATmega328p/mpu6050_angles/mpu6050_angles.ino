/*
 * Example sketch showcasing use of the mpu6050 Arduino library to
 * calculate roll, pitch, and yaw angles.
 */

#include "mpu6050.h"

// Optional: pass an led pin number into the constructor to enable
// the led indicator
MPU6050 mpu6050(LED_BUILTIN);

long unsigned timer;

void setup()
{
    Serial.begin(9600);

    // call calibrate() within setup()
    mpu6050.begin();

    timer = micros();
}

void loop()
{
    // call run() within loop() 50-250 Hz for best results.
    // This method computes the change in orientation between calls
    mpu6050.run();

    // roll() returns an angle in degrees (-180 < roll < 180)
    Serial.print(mpu6050.roll());
    Serial.print(' ');

    // pitch() returns an angle in degrees (-90 < pitch < 90)
    Serial.print(mpu6050.pitch());
    Serial.print(' ');

    // yaw() returns and angle in degrees (-180 < yaw < 180)
    Serial.println(mpu6050.yaw());

    // regulate loop to 200Hz
    while (micros() - timer < 5000);
    timer = micros();
}
