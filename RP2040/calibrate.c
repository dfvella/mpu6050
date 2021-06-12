/*
 * prints the average of many accelerometer readins over usb
 * place mpu6050 on level surface then run. use these values
 * define MPU6050_ACCEL_<X|Y|Z>_LEVEL in mpu6050.h
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "mpu6050.h"

#define NUM_READINGS 2500

#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21

int main() {
    stdio_init_all();

    i2c_init(&i2c0_inst, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    printf("Initializing mpu6050...\n");

    mpu6050_inst_t mpu6050;
    if (mpu6050_init(&mpu6050, &i2c0_inst, PICO_DEFAULT_LED_PIN)) {
        printf("mpu6050 not found\n");
        return 1;
    }

    printf("Sampling mpu6050 for %d seconds...\n", NUM_READINGS / 250);

    mpu6050_data_t data;
    if (mpu6050_avg_reading(&mpu6050, &data, NUM_READINGS)) {
        printf("mpu6050 disconnected\n");
        return 1;
    }

    printf("x: %d, y: %d, z: %d\n",
        data.accel_x,
        data.accel_y,
        data.accel_z - MPU6050_TICKS_PER_G
    );

    return 0;
}
