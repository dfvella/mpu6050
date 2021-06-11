/*
 * print roll, pitch, and yaw angles over usb
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "mpu6050.h"

#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21

int main() {
    stdio_init_all();

    i2c_init(&i2c0_inst, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    mpu6050_inst_t mpu6050;
    mpu6050_init(&mpu6050, &i2c0_inst, PICO_DEFAULT_LED_PIN);

    absolute_time_t timer = get_absolute_time();
    while (1) {
        mpu6050_update_state(&mpu6050);

        printf("roll: %f, pitch: %f, yaw: %f\n",
            mpu6050_get_roll(&mpu6050),
            mpu6050_get_pitch(&mpu6050),
            mpu6050_get_yaw(&mpu6050)
        );

        while (absolute_time_diff_us(timer, get_absolute_time()) < 20000);
        timer = get_absolute_time();
    }
}
