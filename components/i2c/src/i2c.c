#include "i2c.h"

void i2c_init(i2c_master_bus_handle_t *bus_handle) {
   i2c_master_bus_config_t bus_config = {
       .i2c_port = I2C_MASTER_NUM,
       .sda_io_num = I2C_MASTER_SDA_IO,
       .scl_io_num = I2C_MASTER_SCL_IO,
       .clk_source = I2C_CLK_SRC_DEFAULT,
       .glitch_ignore_cnt = 7,
       .flags.enable_internal_pullup = false,
   };
   ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
}
