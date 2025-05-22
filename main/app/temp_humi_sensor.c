/*
 * DHT11 one-wire driver – ESP32-S3 GPIO41 (可自行修改)
 * 修复版本 - 解决时序逻辑问题
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"        // esp_timer_get_time()
#include "rom/ets_sys.h"      // esp_rom_delay_us()
#include "temp_humi_sensor.h"

/*=================== 可调参数 ===================*/
#define DHT_GPIO           13      // 数据线用的 GPIO
#define DHT_DEBUG          1       // 1=开调试 0=关
#define DHT_START_LOW_MS   30      // 起始低电平时长
#define DHT_RESP_WAIT_US   500     // 等待对方拉高/拉低最大 us
#define DHT_BIT_HIGH_TH_US 50      // 高电平>40µs 视为 1
/*================================================*/

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

/* 等待 GPIO 变成指定电平，超时返回 false */
static inline bool wait_for_level(int target_level, int timeout_us)
{
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(DHT_GPIO) != target_level) {
        if ((esp_timer_get_time() - start_time) > timeout_us) {
            return false;
        }
        // 让出CPU避免看门狗复位
        if ((esp_timer_get_time() - start_time) % 10 == 0) {
            esp_rom_delay_us(1);
        }
    }
    return true;
}

/* 读取 5 字节原始数据 */
static int dht11_read_raw(uint8_t data[5])
{
    /* ---------- 启动信号 ---------- */
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(DHT_START_LOW_MS));     // 18-20 ms
    
    gpio_set_level(DHT_GPIO, 1);
    esp_rom_delay_us(30);                            // 20-40 µs
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);   // 释放总线

    /* ---------- DHT 响应序列 ---------- */
    // 1. 等待DHT11拉低（响应开始）
    if (!wait_for_level(0, DHT_RESP_WAIT_US)) { 
        DHT_LOGW("No response (wait low)"); 
        return DHT_TIMEOUT; 
    }
    
    // 2. 等待DHT11拉高（响应低电平阶段结束）
    if (!wait_for_level(1, DHT_RESP_WAIT_US)) { 
        DHT_LOGW("No response (wait high)"); 
        return DHT_TIMEOUT; 
    }
    
    // 3. 等待DHT11再次拉低（准备发送数据）
    if (!wait_for_level(0, DHT_RESP_WAIT_US)) { 
        DHT_LOGW("No response (wait data start)"); 
        return DHT_TIMEOUT; 
    }

    DHT_LOGI("Handshake OK");

    /* ---------- 读取 40 位 ---------- */
    for (int i = 0; i < 5; i++) data[i] = 0;

    for (int i = 0; i < 40; i++) {

        /* (1) 等待位起始高电平（每位开始时DHT11会拉高） */
        if (!wait_for_level(1, DHT_RESP_WAIT_US)) {
            DHT_LOGW("Timeout bit %d (wait high)", i);
            return DHT_TIMEOUT;
        }

        /* (2) 测量高电平持续时间来判断是0还是1 */
        int64_t high_start = esp_timer_get_time();
        
        /* (3) 等待高电平结束（DHT11拉低） */
        if (!wait_for_level(0, DHT_RESP_WAIT_US)) {
            DHT_LOGW("Timeout bit %d (measure high)", i);
            return DHT_TIMEOUT;
        }
        
        int64_t high_duration = esp_timer_get_time() - high_start;

        /* (4) 根据高电平持续时间判断位值 */
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

/* ---------- 外部 API ---------- */

esp_err_t temp_humi_sensor_init(void)
{
    gpio_reset_pin(DHT_GPIO);
    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);  // 初始状态设为输入
    
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

    // 校验和检查
    uint8_t checksum = raw[0] + raw[1] + raw[2] + raw[3];
    if (checksum != raw[4]) {
        DHT_LOGW("Checksum error (calc=0x%02X recv=0x%02X)", checksum, raw[4]);
        return ESP_FAIL;
    }

    // DHT11 数据格式：整数部分
    *humidity    = (float)raw[0];   // 湿度整数部分
    *temperature = (float)raw[2];   // 温度整数部分

    DHT_LOGI("Raw bytes: %02X %02X %02X %02X %02X", raw[0], raw[1], raw[2], raw[3], raw[4]);
    DHT_LOGI("Temperature = %.1f °C, Humidity = %.1f %%", *temperature, *humidity);
    
    return ESP_OK;
}