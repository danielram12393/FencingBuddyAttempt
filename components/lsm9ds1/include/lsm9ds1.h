#ifndef LSM9DS1_H
#define LSM9DS1_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "portmacro.h"

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

typedef struct {
   float x;
   float y;
   float z;
} vec3_t;

esp_err_t lsm9ds1_read_byte(i2c_master_dev_handle_t dev_handle,
                            uint8_t reg_addr, uint8_t *data_buffer);

esp_err_t lsm9ds1_write_byte(i2c_master_dev_handle_t dev_handle,
                             uint8_t reg_addr, uint8_t data);

vec3_t lsm9ds1_get_accl_vec(i2c_master_dev_handle_t dev_handle);

#endif
