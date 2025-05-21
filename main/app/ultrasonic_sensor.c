#include "ultrasonic_sensor.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "ultrasonic";

#define ULTRASONIC_TIMEOUT_US 25000  // 25ms timeout (for up to ~4m range)
#define SOUND_SPEED_ADJUST_FACTOR 0.0343 // Speed of sound in cm/us at 20°C

static uint8_t trig_pin;
static uint8_t echo_pin;

esp_err_t ultrasonic_init(uint8_t trigger_pin, uint8_t echo_pin)
{
    trig_pin = trigger_pin;
    echo_pin = echo_pin;
    
    // Configure trigger pin as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << trig_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configuring trigger pin");
        return ret;
    }
    
    // Configure echo pin as input
    io_conf.pin_bit_mask = (1ULL << echo_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configuring echo pin");
        return ret;
    }
    
    // Set trigger pin low initially
    gpio_set_level(trig_pin, 0);
    
    ESP_LOGI(TAG, "Ultrasonic sensor initialized on trigger=%d, echo=%d", trig_pin, echo_pin);
    return ESP_OK;
}

esp_err_t ultrasonic_measure(float *distance_cm)
{
    if (distance_cm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t echo_start, echo_end, echo_duration;
    int64_t timeout_time;

    // 发送 10μs 触发脉冲
    gpio_set_level(trig_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig_pin, 0);

    // 等待回波上升沿（带超时）
    timeout_time = esp_timer_get_time() + ULTRASONIC_TIMEOUT_US;
    while (gpio_get_level(echo_pin) == 0) {
        if (esp_timer_get_time() > timeout_time) {
            ESP_LOGW(TAG, "Echo rising edge timeout");
            *distance_cm = 0;
            return ESP_ERR_TIMEOUT;
        }
        taskYIELD();  // 让出调度，切换到 Wi-Fi、LVGL 等高优先级任务
    }
    echo_start = esp_timer_get_time();

    // 等待回波下降沿（带超时）
    timeout_time = echo_start + ULTRASONIC_TIMEOUT_US;
    while (gpio_get_level(echo_pin) == 1) {
        if (esp_timer_get_time() > timeout_time) {
            ESP_LOGW(TAG, "Echo falling edge timeout");
            *distance_cm = 0;
            return ESP_ERR_TIMEOUT;
        }
        taskYIELD();  // 持续让出，避免长时间占用 CPU
    }
    echo_end = esp_timer_get_time();

    // 计算距离：声波往返时间 × 声速（cm/μs）除以 2
    echo_duration = echo_end - echo_start;
    *distance_cm = (echo_duration * SOUND_SPEED_ADJUST_FACTOR) / 2.0;

    ESP_LOGD(TAG, "Distance: %.2f cm (pulse: %lld μs)",
             *distance_cm, echo_duration);
    return ESP_OK;
}


bool ultrasonic_obstacle_detected(float threshold_cm)
{
    float distance;
    esp_err_t ret = ultrasonic_measure(&distance);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to measure distance: %s", esp_err_to_name(ret));
        return false; // Assume no obstacle on error
    }
    
    // Check if distance is within threshold
    bool obstacle = (distance > 0 && distance <= threshold_cm);
    
    if (obstacle) {
        ESP_LOGI(TAG, "Obstacle detected at %.2f cm (threshold: %.2f cm)", distance, threshold_cm);
    }
    
    return obstacle;
}