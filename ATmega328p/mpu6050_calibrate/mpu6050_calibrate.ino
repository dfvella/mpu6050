/*
 * Sketch for calibraing the mpu6050 imu
 */

// ***** CALIBRATION INTRUSCTIONS *****
// - Place mpu6050 on a level surface
// - Upload the sketch and open serial monitor
// - After sketch completes, enter values that appear in the serial
//   monitor into the macro definitions at the top of imu.h

#include "mpu6050.h"

MPU6050 mpu6050(LED_BUILTIN);

void setup()
{
    Serial.begin(9600);
    mpu6050.calibrate_accel();
}

void loop() { }
