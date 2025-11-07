/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected
   over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"
#include "sdkconfig.h"
#include <stdio.h>

static const char *TAG = "example";

#define I2C_MASTER_SCL_IO                                                      \
   CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO                                                      \
   CONFIG_I2C_MASTER_SDA         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ                                                     \
   CONFIG_I2C_MASTER_FREQUENCY      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define LSM9DS1_I2C_ADDRESS 0x6B

#define LSM9DS1_I2C_WHO_AM_I 0x0F

#define LSM9DS1_CTRL_REG6_XL 0x20
#define LSM9DS1_CTRL_REG8 0x22

#define LSM9DS1_CTRL_REG2_M 0x21

typedef struct {
   int16_t x;
   int16_t y;
   int16_t z;
} int16_vec3_t;

static esp_err_t lsm9ds1_read_byte(i2c_master_dev_handle_t dev_handle,
                                   uint8_t reg_addr, uint8_t *data_buffer) {
   uint8_t write_buf[1] = {reg_addr};
   return i2c_master_transmit_receive(
       dev_handle, write_buf, sizeof(write_buf), data_buffer, 1,
       I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t lsm9ds1_write_byte(i2c_master_dev_handle_t dev_handle,
                                    uint8_t reg_addr, uint8_t data) {
   uint8_t write_buf[2] = {reg_addr, data};
   return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf),
                              I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static int16_vec3_t lsm9ds1_get_accl_vec(i2c_master_dev_handle_t dev_handle) {
   int16_vec3_t ret_vec;

   uint8_t most_significant;
   uint8_t least_significant;

   ESP_ERROR_CHECK(lsm9ds1_read_byte(dev_handle, 0x28, &least_significant));
   ESP_ERROR_CHECK(lsm9ds1_read_byte(dev_handle, 0x29, &most_significant));

   ret_vec.x = (most_significant << 8) | least_significant;

   ESP_ERROR_CHECK(lsm9ds1_read_byte(dev_handle, 0x2A, &least_significant));
   ESP_ERROR_CHECK(lsm9ds1_read_byte(dev_handle, 0x2B, &most_significant));

   ret_vec.y = (most_significant << 8) | least_significant;

   ESP_ERROR_CHECK(lsm9ds1_read_byte(dev_handle, 0x2C, &least_significant));
   ESP_ERROR_CHECK(lsm9ds1_read_byte(dev_handle, 0x2D, &most_significant));

   ret_vec.z = (most_significant << 8) | least_significant;

   return ret_vec;
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle,
                            i2c_master_dev_handle_t *dev_handle) {
   i2c_master_bus_config_t bus_config = {
       .i2c_port = I2C_MASTER_NUM,
       .sda_io_num = I2C_MASTER_SDA_IO,
       .scl_io_num = I2C_MASTER_SCL_IO,
       .clk_source = I2C_CLK_SRC_DEFAULT,
       .glitch_ignore_cnt = 7,
       .flags.enable_internal_pullup = false,
   };
   ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

   i2c_device_config_t dev_config = {
       .dev_addr_length = I2C_ADDR_BIT_LEN_7,
       .device_address = LSM9DS1_I2C_ADDRESS,
       .scl_speed_hz = I2C_MASTER_FREQ_HZ,
   };
   ESP_ERROR_CHECK(
       i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

void app_main(void) {
   i2c_master_bus_handle_t bus_handle;
   i2c_master_dev_handle_t dev_handle;
   i2c_master_init(&bus_handle, &dev_handle);
   ESP_LOGI(TAG, "I2C initialized successfully");

   // Turn on accelerometer
   ESP_ERROR_CHECK(lsm9ds1_write_byte(dev_handle, LSM9DS1_CTRL_REG8, 0x5));

   ESP_ERROR_CHECK(lsm9ds1_write_byte(dev_handle, LSM9DS1_CTRL_REG6_XL, 0x70));

   while (true) {
      int16_vec3_t vec = lsm9ds1_get_accl_vec(dev_handle);
      printf("X:%6d Y:%6d Z:%6d\n", vec.x, vec.y, vec.z);
      vTaskDelay(10 / portTICK_PERIOD_MS);
   }
}
