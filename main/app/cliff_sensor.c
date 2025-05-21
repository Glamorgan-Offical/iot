#include "cliff_sensor.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "robot_motion.h"

static const char *TAG = "cliff_sensor";

#define CLIFF_SENSOR_PIN 40
#define CLIFF_CHECK_INTERVAL_MS 50
#define DEBOUNCE_COUNT 1  // 需要连续3次检测到才确认是悬崖

static bool cliff_detection_enabled = false;
static bool cliff_detected = false;
static TimerHandle_t cliff_timer = NULL;
static uint8_t debounce_counter = 0;

// 防抖函数 - 需要连续多次读取相同状态才确认
static bool debounced_read(void)
{
    // 读取当前状态 (0 = 检测到悬崖)
    bool current_read = (gpio_get_level(CLIFF_SENSOR_PIN) == 1);
    
    if (current_read) {
        // 可能检测到悬崖，增加计数器
        if (debounce_counter < DEBOUNCE_COUNT) {
            debounce_counter++;
        }
    } else {
        // 未检测到悬崖，重置计数器
        debounce_counter = 0;
    }
    
    // 只有当计数器达到阈值才返回true
    return (debounce_counter >= DEBOUNCE_COUNT);
}

static void IRAM_ATTR cliff_sensor_isr(void* arg)
{
    // 在ISR中不做防抖处理，仅记录事件
    bool raw_state = (gpio_get_level(CLIFF_SENSOR_PIN) == 0);
    
    // 只有从安全到悬崖的转变才在ISR中处理
    if (raw_state && !cliff_detected && cliff_detection_enabled) {
        // 不在ISR中直接调用robot_stop()，而是标记状态
        // 让定时器回调处理，这样更安全
        debounce_counter++;
    }
}

static void cliff_check_timer_callback(TimerHandle_t xTimer)
{
    if (cliff_detection_enabled) {
        // 使用防抖动读取
        bool real_cliff_state = debounced_read();
        
        if (real_cliff_state != cliff_detected) {
            // 状态变化
            cliff_detected = real_cliff_state;
            
            if (cliff_detected) {
                // 确认检测到悬崖
                robot_stop();
                ESP_LOGI(TAG, "Cliff detected! Robot stopped");
            }
        }
    } else {
        // 检测禁用时重置防抖计数器
        debounce_counter = 0;
    }
}

void cliff_sensor_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CLIFF_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // 尝试读取初始状态
    debounce_counter = 0;
    cliff_detected = false;
    
    // 安装GPIO中断服务
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CLIFF_SENSOR_PIN, cliff_sensor_isr, NULL);
    
    // 创建定时器做防抖动处理
    cliff_timer = xTimerCreate(
        "CliffTimer",
        50,
        pdTRUE,
        NULL,
        cliff_check_timer_callback
    );
    
    if (cliff_timer != NULL) {
        xTimerStart(cliff_timer, 0);
    }
    
    ESP_LOGI(TAG, "Cliff sensor initialized on pin %d, with debounce count %d", 
             CLIFF_SENSOR_PIN, DEBOUNCE_COUNT);
}

void cliff_detection_enable(bool enable)
{
    cliff_detection_enabled = enable;
    if (!enable) {
        // 重置状态
        debounce_counter = 0;
        cliff_detected = false;
    }
    ESP_LOGI(TAG, "Cliff detection %s", enable ? "enabled" : "disabled");
}

bool is_cliff_detected(void)
{
    return cliff_detected;
}