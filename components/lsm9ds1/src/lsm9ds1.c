#include "lsm9ds1.h"

esp_err_t lsm9ds1_read_byte(i2c_master_dev_handle_t dev_handle,
                            uint8_t reg_addr, uint8_t *data_buffer) {
   uint8_t write_buf[1] = {reg_addr};
   return i2c_master_transmit_receive(
       dev_handle, write_buf, sizeof(write_buf), data_buffer, 1,
       I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t lsm9ds1_write_byte(i2c_master_dev_handle_t dev_handle,
                             uint8_t reg_addr, uint8_t data) {
   uint8_t write_buf[2] = {reg_addr, data};
   return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf),
                              I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

vec3_t lsm9ds1_get_accl_vec(i2c_master_dev_handle_t dev_handle) {
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

   vec3_t float_vec = {.x = ret_vec.x * .000732,
                       .y = ret_vec.y * .000732,
                       .z = ret_vec.z * .000732};

   return float_vec;
}
