#ifndef I2C_H
#define I2C_H

#include "driver/i2c_master.h"

#define I2C_MASTER_SCL_IO 0
#define I2C_MASTER_SDA_IO 1
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

void i2c_init(i2c_master_bus_handle_t *bus_handle);

#endif
