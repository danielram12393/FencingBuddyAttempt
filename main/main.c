#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "lsm9ds1.h"
#include "portmacro.h"
#include <stdio.h>

static const char *TAG = "INIT";

i2c_master_bus_handle_t bus_handle;
lsm9ds1_t lsm;

void app_main(void) {
   i2c_init(&bus_handle);
   lsm9ds1_init_i2c(&bus_handle, &lsm);
   ESP_LOGI(TAG, "I2C initialized successfully");
   lsm9ds1_init_device(&lsm);
   ESP_LOGI(TAG, "LSM9DS1 initialized successfully");

   while (true) {
      vec3_t xl_vec = lsm9ds1_get_accl_vec(&lsm);
      vec3_t gyro_vec = lsm9ds1_get_gyro_vec(&lsm);
      vec3_t mag_vec = lsm9ds1_get_mag_vec(&lsm);
      printf("==============================================\n");
      printf("Accel X:%8.3f Y:%8.3f Z:%8.3f\n", xl_vec.x, xl_vec.y, xl_vec.z);
      printf("Gyro  X:%8.3f Y:%8.3f Z:%8.3f\n", gyro_vec.x, gyro_vec.y,
             gyro_vec.z);
      printf("Mag   X:%8.3f Y:%8.3f Z:%8.3f\n", mag_vec.x, mag_vec.y,
             mag_vec.z);
      vTaskDelay(100 / portTICK_PERIOD_MS);
   }
}
