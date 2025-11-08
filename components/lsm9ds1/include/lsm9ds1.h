#ifndef LSM9DS1_H
#define LSM9DS1_H

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_dsp.h"
#include "esp_err.h"
#include "portmacro.h"

// clang-format off
#define I2C_MASTER_FREQ_HZ           400000
#define I2C_MASTER_TIMEOUT_MS        1000

#define LSM9DS1_XL_CONVERSION        .000732f
#define LSM9DS1_GYRO_CONVERSION      .07f
#define LSM9DS1_MAG_CONVERSION       .00014f

/*
 *  I2C Device Addresses
 */

#define LSM9DS1_GYRO_XL_ADDRESS      0x6B
#define LSM9DS1_MAG_ADDRESS          0x1E

/*
 *  Gyro and Accelerometer Register Addresses
 */

#define LSM9DS1_ACT_THS              0x04
#define LSM9DS1_ACT_DUR              0x05
#define LSM9DS1_INT_GEN_CFG_XL       0x06
#define LSM9DS1_INT_GEN_THS_X_XL     0x07
#define LSM9DS1_INT_GEN_THS_Y_XL     0x08
#define LSM9DS1_INT_GEN_THS_Z_XL     0x09
#define LSM9DS1_INT_GEN_DUR_XL       0x0A
#define LSM9DS1_REFERENCE_G          0x0B
#define LSM9DS1_INT1_CTRL            0x0C
#define LSM9DS1_INT2_CTRL            0x0D
#define LSM9DS1_WHO_AM_I             0x0F
#define LSM9DS1_CTRL_REG1_G          0x10
#define LSM9DS1_CTRL_REG2_G          0x11
#define LSM9DS1_CTRL_REG3_G          0x12
#define LSM9DS1_ORIENT_CFG_G         0x13
#define LSM9DS1_INT_GEN_SRC_G        0x14
#define LSM9DS1_OUT_TEMP_L           0x15
#define LSM9DS1_OUT_TEMP_H           0x16
#define LSM9DS1_STATUS_REG           0x17
#define LSM9DS1_OUT_X_L_G            0x18
#define LSM9DS1_OUT_X_H_G            0x19
#define LSM9DS1_OUT_Y_L_G            0x1A
#define LSM9DS1_OUT_Y_H_G            0x1B
#define LSM9DS1_OUT_Z_L_G            0x1C
#define LSM9DS1_OUT_Z_H_G            0x1D
#define LSM9DS1_CTRL_REG4            0x1E
#define LSM9DS1_CTRL_REG5_XL         0x1F
#define LSM9DS1_CTRL_REG6_XL         0x20
#define LSM9DS1_CTRL_REG7_XL         0x21
#define LSM9DS1_CTRL_REG8            0x22
#define LSM9DS1_CTRL_REG9            0x23
#define LSM9DS1_CTRL_REG10           0x24
#define LSM9DS1_INT_GEN_SRC_XL       0x26
#define LSM9DS1_STATUS_REG_XL        0x27
#define LSM9DS1_OUT_X_L_XL           0x28
#define LSM9DS1_OUT_X_H_XL           0x29
#define LSM9DS1_OUT_Y_L_XL           0x2A
#define LSM9DS1_OUT_Y_H_XL           0x2B
#define LSM9DS1_OUT_Z_L_XL           0x2C
#define LSM9DS1_OUT_Z_H_XL           0x2D
#define LSM9DS1_FIFO_CTRL            0x2E
#define LSM9DS1_FIFO_SRC             0x2F
#define LSM9DS1_INT_GEN_CFG_G        0x30
#define LSM9DS1_INT_GEN_THS_XH_G     0x31
#define LSM9DS1_INT_GEN_THS_XL_G     0x32
#define LSM9DS1_INT_GEN_THS_YH_G     0x33
#define LSM9DS1_INT_GEN_THS_YL_G     0x34
#define LSM9DS1_INT_GEN_THS_ZH_G     0x35
#define LSM9DS1_INT_GEN_THS_ZL_G     0x36
#define LSM9DS1_INT_GEN_DUR_G        0x37

/*
*  Magnometer Register Addresses
*/

#define LSM9DS1_OFFSET_X_REG_L_M     0x05
#define LSM9DS1_OFFSET_X_REG_H_M     0x06
#define LSM9DS1_OFFSET_Y_REG_L_M     0x07
#define LSM9DS1_OFFSET_Y_REG_H_M     0x08
#define LSM9DS1_OFFSET_Z_REG_L_M     0x09
#define LSM9DS1_OFFSET_Z_REG_H_M     0x0A
#define LSM9DS1_WHO_AM_I_M           0x0F
#define LSM9DS1_CTRL_REG1_M          0x20
#define LSM9DS1_CTRL_REG2_M          0x21
#define LSM9DS1_CTRL_REG3_M          0x22
#define LSM9DS1_CTRL_REG4_M          0x23
#define LSM9DS1_CTRL_REG5_M          0x24
#define LSM9DS1_STATUS_REG_M         0x27
#define LSM9DS1_OUT_X_L_M            0x28
#define LSM9DS1_OUT_X_H_M            0x29
#define LSM9DS1_OUT_Y_L_M            0x2A
#define LSM9DS1_OUT_Y_H_M            0x2B
#define LSM9DS1_OUT_Z_L_M            0x2C
#define LSM9DS1_OUT_Z_H_M            0x2D
#define LSM9DS1_INT_CFG_M            0x30
#define LSM9DS1_INT_SRC_M            0x31
#define LSM9DS1_INT_THS_L_M          0x32
#define LSM9DS1_INT_THS_H_M          0x33
//clang-format on

typedef struct {
   int16_t x;
   int16_t y;
   int16_t z;
} int16_vec3_t;

typedef union {
   struct {
      float x;
      float y;
      float z;
   };
   float buffer[3];
} vec3_t;

typedef struct {
   i2c_master_dev_handle_t gyro_xl_handle;
   i2c_master_dev_handle_t mag_handle;
} lsm9ds1_t;

void lsm9ds1_init_i2c(i2c_master_bus_handle_t *bus_handle, lsm9ds1_t *lsm);
void lsm9ds1_init_device(lsm9ds1_t *lsm);

esp_err_t lsm9ds1_gyro_xl_read_byte(lsm9ds1_t *lsm,
                            uint8_t reg_addr, uint8_t *data_buffer);

esp_err_t lsm9ds1_gyro_xl_write_byte(lsm9ds1_t *lsm,
                             uint8_t reg_addr, uint8_t data);

esp_err_t lsm9ds1_mag_read_byte(lsm9ds1_t *lsm,
                            uint8_t reg_addr, uint8_t *data_buffer);

esp_err_t lsm9ds1_mag_write_byte(lsm9ds1_t *lsm,
                             uint8_t reg_addr, uint8_t data);

vec3_t lsm9ds1_get_accl_vec(lsm9ds1_t *lsm);
vec3_t lsm9ds1_get_gyro_vec(lsm9ds1_t *lsm);
vec3_t lsm9ds1_get_mag_vec(lsm9ds1_t *lsm);

#endif
