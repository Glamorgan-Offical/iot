#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t temp_humi_sensor_init(void);
esp_err_t temp_humi_sensor_read(float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif