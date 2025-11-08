#include "lsm9ds1.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

void lsm9ds1_init_i2c(i2c_master_bus_handle_t *bus_handle, lsm9ds1_t *lsm) {

   i2c_device_config_t gyro_xl_config = {
       .dev_addr_length = I2C_ADDR_BIT_LEN_7,
       .device_address = LSM9DS1_GYRO_XL_ADDRESS,
       .scl_speed_hz = I2C_MASTER_FREQ_HZ,
   };
   i2c_master_dev_handle_t gyro_xl_handle;

   i2c_device_config_t mag_config = {
       .dev_addr_length = I2C_ADDR_BIT_LEN_7,
       .device_address = LSM9DS1_MAG_ADDRESS,
       .scl_speed_hz = I2C_MASTER_FREQ_HZ,
   };
   i2c_master_dev_handle_t mag_handle;

   ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &gyro_xl_config,
                                             &gyro_xl_handle));

   ESP_ERROR_CHECK(
       i2c_master_bus_add_device(*bus_handle, &mag_config, &mag_handle));

   lsm->gyro_xl_handle = gyro_xl_handle;
   lsm->mag_handle = mag_handle;
}

void lsm9ds1_init_device(lsm9ds1_t *lsm) {
   ESP_ERROR_CHECK(lsm9ds1_gyro_xl_write_byte(lsm, LSM9DS1_CTRL_REG8, 0x5));

   ESP_ERROR_CHECK(lsm9ds1_gyro_xl_write_byte(lsm, LSM9DS1_CTRL_REG1_G, 0xBB));
   ESP_ERROR_CHECK(lsm9ds1_gyro_xl_write_byte(lsm, LSM9DS1_CTRL_REG6_XL, 0x08));

   ESP_ERROR_CHECK(lsm9ds1_mag_write_byte(lsm, LSM9DS1_CTRL_REG3_M, 0x00));
}

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

esp_err_t lsm9ds1_gyro_xl_read_byte(lsm9ds1_t *lsm, uint8_t reg_addr,
                                    uint8_t *data_buffer) {
   return lsm9ds1_read_byte(lsm->gyro_xl_handle, reg_addr, data_buffer);
}

esp_err_t lsm9ds1_gyro_xl_write_byte(lsm9ds1_t *lsm, uint8_t reg_addr,
                                     uint8_t data) {
   return lsm9ds1_write_byte(lsm->gyro_xl_handle, reg_addr, data);
}

esp_err_t lsm9ds1_mag_read_byte(lsm9ds1_t *lsm, uint8_t reg_addr,
                                uint8_t *data_buffer) {
   return lsm9ds1_read_byte(lsm->mag_handle, reg_addr, data_buffer);
}

esp_err_t lsm9ds1_mag_write_byte(lsm9ds1_t *lsm, uint8_t reg_addr,
                                 uint8_t data) {
   return lsm9ds1_write_byte(lsm->mag_handle, reg_addr, data);
}

vec3_t lsm9ds1_get_accl_vec(lsm9ds1_t *lsm) {
   int16_vec3_t vec;

   uint8_t most_significant;
   uint8_t least_significant;

   ESP_ERROR_CHECK(lsm9ds1_gyro_xl_read_byte(lsm, 0x28, &least_significant));
   ESP_ERROR_CHECK(lsm9ds1_gyro_xl_read_byte(lsm, 0x29, &most_significant));

   vec.x = (most_significant << 8) | least_significant;

   ESP_ERROR_CHECK(lsm9ds1_gyro_xl_read_byte(lsm, 0x2A, &least_significant));
   ESP_ERROR_CHECK(lsm9ds1_gyro_xl_read_byte(lsm, 0x2B, &most_significant));

   vec.y = (most_significant << 8) | least_significant;

   ESP_ERROR_CHECK(lsm9ds1_gyro_xl_read_byte(lsm, 0x2C, &least_significant));
   ESP_ERROR_CHECK(lsm9ds1_gyro_xl_read_byte(lsm, 0x2D, &most_significant));

   vec.z = (most_significant << 8) | least_significant;

   vec3_t float_vec = {.x = vec.x, .y = vec.y, .z = vec.z};
   vec3_t scaled_vec;
   dsps_mulc_f32_ansi(float_vec.buffer, scaled_vec.buffer, 3,
                      LSM9DS1_XL_CONVERSION, 1, 1);

   return scaled_vec;
}

vec3_t lsm9ds1_get_gyro_vec(lsm9ds1_t *lsm) {
   int16_vec3_t vec;

   uint8_t most_significant;
   uint8_t least_significant;

   ESP_ERROR_CHECK(
       lsm9ds1_gyro_xl_read_byte(lsm, LSM9DS1_OUT_X_L_G, &least_significant));
   ESP_ERROR_CHECK(
       lsm9ds1_gyro_xl_read_byte(lsm, LSM9DS1_OUT_X_H_G, &most_significant));

   vec.x = (most_significant << 8) | least_significant;

   ESP_ERROR_CHECK(
       lsm9ds1_gyro_xl_read_byte(lsm, LSM9DS1_OUT_Y_L_G, &least_significant));
   ESP_ERROR_CHECK(
       lsm9ds1_gyro_xl_read_byte(lsm, LSM9DS1_OUT_Y_H_G, &most_significant));

   vec.y = (most_significant << 8) | least_significant;

   ESP_ERROR_CHECK(
       lsm9ds1_gyro_xl_read_byte(lsm, LSM9DS1_OUT_Z_L_G, &least_significant));
   ESP_ERROR_CHECK(
       lsm9ds1_gyro_xl_read_byte(lsm, LSM9DS1_OUT_Z_H_G, &most_significant));

   vec.z = (most_significant << 8) | least_significant;

   vec3_t float_vec = {.x = vec.x, .y = vec.y, .z = vec.z};
   vec3_t scaled_vec;
   dsps_mulc_f32_ansi(float_vec.buffer, scaled_vec.buffer, 3,
                      LSM9DS1_GYRO_CONVERSION, 1, 1);

   return scaled_vec;
}

vec3_t lsm9ds1_get_mag_vec(lsm9ds1_t *lsm) {
   int16_vec3_t vec;

   uint8_t most_significant;
   uint8_t least_significant;

   ESP_ERROR_CHECK(lsm9ds1_read_byte(lsm->mag_handle, LSM9DS1_OUT_X_L_M,
                                     &least_significant));
   ESP_ERROR_CHECK(lsm9ds1_read_byte(lsm->mag_handle, LSM9DS1_OUT_X_H_M,
                                     &most_significant));

   vec.x = (most_significant << 8) | least_significant;

   ESP_ERROR_CHECK(lsm9ds1_read_byte(lsm->mag_handle, LSM9DS1_OUT_Y_L_M,
                                     &least_significant));
   ESP_ERROR_CHECK(lsm9ds1_read_byte(lsm->mag_handle, LSM9DS1_OUT_Y_H_M,
                                     &most_significant));

   vec.y = (most_significant << 8) | least_significant;

   ESP_ERROR_CHECK(lsm9ds1_read_byte(lsm->mag_handle, LSM9DS1_OUT_Z_L_M,
                                     &least_significant));
   ESP_ERROR_CHECK(lsm9ds1_read_byte(lsm->mag_handle, LSM9DS1_OUT_Z_H_M,
                                     &most_significant));

   vec.z = (most_significant << 8) | least_significant;

   vec3_t float_vec = {.x = vec.x, .y = vec.y, .z = vec.z};
   vec3_t scaled_vec;
   dsps_mulc_f32_ansi(float_vec.buffer, scaled_vec.buffer, 3,
                      LSM9DS1_MAG_CONVERSION, 1, 1);

   return scaled_vec;
}
