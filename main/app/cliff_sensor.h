#ifndef CLIFF_SENSOR_H
#define CLIFF_SENSOR_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

void cliff_sensor_init(void);
void cliff_detection_enable(bool enable);
bool is_cliff_detected(void);

#endif // CLIFF_SENSOR_H