#include "cliff_sensor.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "robot_motion.h"

static const char *TAG = "cliff_sensor";

#define CLIFF_SENSOR_PIN 40
#define CLIFF_CHECK_INTERVAL_MS 50
#define DEBOUNCE_COUNT 1  

static bool cliff_detection_enabled = false;
static bool cliff_detected = false;
static TimerHandle_t cliff_timer = NULL;
static uint8_t debounce_counter = 0;

// Debounce function - requires multiple consecutive readings of the same state to confirm
static bool debounced_read(void)
{
    // Read current state (0 = cliff detected)
    bool current_read = (gpio_get_level(CLIFF_SENSOR_PIN) == 1);
    
    if (current_read) {
        // Possible cliff detected, increment counter
        if (debounce_counter < DEBOUNCE_COUNT) {
            debounce_counter++;
        }
    } else {
        // No cliff detected, reset counter
        debounce_counter = 0;
    }
    
    // Only return true when counter reaches threshold
    return (debounce_counter >= DEBOUNCE_COUNT);
}

static void IRAM_ATTR cliff_sensor_isr(void* arg)
{
    // No debouncing in ISR, just record the event
    bool raw_state = (gpio_get_level(CLIFF_SENSOR_PIN) == 0);
    
    // Only process transition from safe to cliff in ISR
    if (raw_state && !cliff_detected && cliff_detection_enabled) {
        debounce_counter++;
    }
}

static void cliff_check_timer_callback(TimerHandle_t xTimer)
{
    if (cliff_detection_enabled) {
        // Use debounced reading
        bool real_cliff_state = debounced_read();
        
        if (real_cliff_state != cliff_detected) {
            // State change
            cliff_detected = real_cliff_state;
            
            if (cliff_detected) {
                // Confirmed cliff detected - use pause instead of stop
                robot_pause();
                ESP_LOGI(TAG, "Cliff detected! Robot paused");
            }
        }
    } else {
        // Reset debounce counter when detection is disabled
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
    
    // Try to read initial state
    debounce_counter = 0;
    cliff_detected = false;
    
    // Install GPIO interrupt service
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CLIFF_SENSOR_PIN, cliff_sensor_isr, NULL);
    
    // Create timer for debounce processing
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
        // Reset state
        debounce_counter = 0;
        cliff_detected = false;
    }
    ESP_LOGI(TAG, "Cliff detection %s", enable ? "enabled" : "disabled");
}

bool is_cliff_detected(void)
{
    return cliff_detected;
}