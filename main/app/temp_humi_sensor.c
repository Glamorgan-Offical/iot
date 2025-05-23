/*
 * DHT11 one-wire driver – ESP32-S3

 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"       
#include "rom/ets_sys.h"     
#include "temp_humi_sensor.h"


#define DHT_GPIO           13     
#define DHT_DEBUG          1       
#define DHT_START_LOW_MS   30      
#define DHT_RESP_WAIT_US   500     
#define DHT_BIT_HIGH_TH_US 50      

#define DHT_OK             0
#define DHT_TIMEOUT       -1
#define DHT_CHECKSUM      -2

static const char *TAG = "DHT11";

#if DHT_DEBUG
#define DHT_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#define DHT_LOGW(...) ESP_LOGW(TAG, __VA_ARGS__)
#else
#define DHT_LOGI(...)
#define DHT_LOGW(...)
#endif

static inline bool wait_for_level(int target_level, int timeout_us)
{
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(DHT_GPIO) != target_level) {
        if ((esp_timer_get_time() - start_time) > timeout_us) {
            return false;
        }
        if ((esp_timer_get_time() - start_time) % 10 == 0) {
            esp_rom_delay_us(1);
        }
    }
    return true;
}

static int dht11_read_raw(uint8_t data[5])
{
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(DHT_START_LOW_MS));     
    
    gpio_set_level(DHT_GPIO, 1);
    esp_rom_delay_us(30);                           
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);   

    if (!wait_for_level(0, DHT_RESP_WAIT_US)) { 
        DHT_LOGW("No response (wait low)"); 
        return DHT_TIMEOUT; 
    }
    
    if (!wait_for_level(1, DHT_RESP_WAIT_US)) { 
        DHT_LOGW("No response (wait high)"); 
        return DHT_TIMEOUT; 
    }
    
    if (!wait_for_level(0, DHT_RESP_WAIT_US)) { 
        DHT_LOGW("No response (wait data start)"); 
        return DHT_TIMEOUT; 
    }

    DHT_LOGI("Handshake OK");

    for (int i = 0; i < 5; i++) data[i] = 0;

    for (int i = 0; i < 40; i++) {

        if (!wait_for_level(1, DHT_RESP_WAIT_US)) {
            DHT_LOGW("Timeout bit %d (wait high)", i);
            return DHT_TIMEOUT;
        }

        int64_t high_start = esp_timer_get_time();
        
        if (!wait_for_level(0, DHT_RESP_WAIT_US)) {
            DHT_LOGW("Timeout bit %d (measure high)", i);
            return DHT_TIMEOUT;
        }
        
        int64_t high_duration = esp_timer_get_time() - high_start;

        uint8_t bit_val = (high_duration > DHT_BIT_HIGH_TH_US) ? 1 : 0;
        int byte_idx = i / 8;
        int bit_idx  = 7 - (i % 8);
        if (bit_val) {
            data[byte_idx] |= (1 << bit_idx);
        }

    #if DHT_DEBUG
        DHT_LOGI("bit%-2d high=%lld us => %d", i, high_duration, bit_val);
    #endif
    }
    
    return DHT_OK;
}


esp_err_t temp_humi_sensor_init(void)
{
    gpio_reset_pin(DHT_GPIO);
    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT); 
    
    DHT_LOGI("DHT11 initialized on GPIO %d", DHT_GPIO);
    return ESP_OK;
}

esp_err_t temp_humi_sensor_read(float *temperature, float *humidity)
{
    if (!temperature || !humidity) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t raw[5] = {0};
    int res = dht11_read_raw(raw);
    if (res != DHT_OK) {
        DHT_LOGW("Failed to read raw data");
        return ESP_FAIL;
    }

    uint8_t checksum = raw[0] + raw[1] + raw[2] + raw[3];
    if (checksum != raw[4]) {
        DHT_LOGW("Checksum error (calc=0x%02X recv=0x%02X)", checksum, raw[4]);
        return ESP_FAIL;
    }

    *humidity    = (float)raw[0];   
    *temperature = (float)raw[2];   

    DHT_LOGI("Raw bytes: %02X %02X %02X %02X %02X", raw[0], raw[1], raw[2], raw[3], raw[4]);
    DHT_LOGI("Temperature = %.1f °C, Humidity = %.1f %%", *temperature, *humidity);
    
    return ESP_OK;
}