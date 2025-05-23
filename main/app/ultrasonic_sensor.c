#include "ultrasonic_sensor.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <math.h>
#include "esp_check.h" 
static const char *TAG = "ultrasonic";

#define ULTRASONIC_TIMEOUT_US 25000  // 25ms timeout (for up to ~4m range)
#define SOUND_SPEED_ADJUST_FACTOR 0.0343 // Speed of sound in cm/us at 20°C

static uint8_t s_trig_pin;
static uint8_t s_echo_pin;

esp_err_t ultrasonic_init(uint8_t trigger_gpio, uint8_t echo_gpio)
{
    s_trig_pin = trigger_gpio;
    s_echo_pin = echo_gpio;

    /************ Trig: Push-pull output, disable pull-up/down ************/
    gpio_config_t io_conf = {0};
    io_conf.pin_bit_mask  = BIT64(s_trig_pin);     
    io_conf.mode          = GPIO_MODE_OUTPUT;
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Trig pin cfg failed");
    gpio_set_level(s_trig_pin, 0);

    /************ Echo: Input, pull-down enabled ************/
    io_conf.pin_bit_mask  = BIT64(s_echo_pin);
    io_conf.mode          = GPIO_MODE_INPUT;
    io_conf.pull_down_en  = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Echo pin cfg failed");

    ESP_LOGI(TAG, "Ultrasonic init OK  (Trig=%d  Echo=%d)", s_trig_pin, s_echo_pin);
    return ESP_OK;
}

esp_err_t ultrasonic_measure(float *distance_cm)
{
    if (distance_cm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (gpio_get_level(s_echo_pin) != 0) {
        ESP_LOGW(TAG, "Echo pin not at low level initially, check hardware");
        return ESP_ERR_INVALID_STATE;
    }

    int64_t echo_start, echo_end;
    int64_t timeout_time;

    gpio_set_level(s_trig_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(s_trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(s_trig_pin, 0);

    timeout_time = esp_timer_get_time() + ULTRASONIC_TIMEOUT_US;
    while (gpio_get_level(s_echo_pin) == 0) {
        if (esp_timer_get_time() > timeout_time) {
            return ESP_ERR_TIMEOUT;
        }
    }
    echo_start = esp_timer_get_time();

    timeout_time = echo_start + ULTRASONIC_TIMEOUT_US;
    while (gpio_get_level(s_echo_pin) == 1) {
        if (esp_timer_get_time() > timeout_time) {
            return ESP_ERR_TIMEOUT;
        }
    }
    echo_end = esp_timer_get_time();

    int64_t echo_duration = echo_end - echo_start;
    *distance_cm = (echo_duration * SOUND_SPEED_ADJUST_FACTOR) / 2.0;

    ESP_LOGD(TAG, "Distance: %.2f cm (pulse: %lld μs)", *distance_cm, echo_duration);
    return ESP_OK;
}


bool ultrasonic_obstacle_detected(float threshold_cm)
{
    float distance;
    esp_err_t ret = ultrasonic_measure(&distance);
    
    if (ret != ESP_OK) {
        return false; 
    }
    
    // Check if distance is within threshold
    bool obstacle = (distance > 0 && distance <= threshold_cm);
    
    if (obstacle) {
        ESP_LOGI(TAG, "Obstacle detected at %.2f cm (threshold: %.2f cm)", distance, threshold_cm);
    }
    
    return obstacle;
}