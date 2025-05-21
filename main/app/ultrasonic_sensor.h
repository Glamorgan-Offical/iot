#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

esp_err_t ultrasonic_init(uint8_t trigger_pin, uint8_t echo_pin);

esp_err_t ultrasonic_measure(float *distance_cm);

bool ultrasonic_obstacle_detected(float threshold_cm);

#endif // ULTRASONIC_SENSOR_H