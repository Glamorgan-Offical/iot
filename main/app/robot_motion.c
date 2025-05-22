#include "robot_motion.h"
#include "cliff_sensor.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ultrasonic_sensor.h"
#include "driver/ledc.h"  

static const char *TAG = "robot_motion";

// 四路电机引脚和PWM通道定义
#define A_PWM_PIN  10
#define A_IN1_PIN  14
#define A_IN2_PIN  11
#define A_LEDC_CH  LEDC_CHANNEL_0

// #define B_PWM_PIN  13
// #define B_IN1_PIN  9
// #define B_IN2_PIN  12
// #define B_LEDC_CH  LEDC_CHANNEL_3

// #define C_PWM_PIN  42
// #define C_IN1_PIN  20
// #define C_IN2_PIN  39
// #define C_LEDC_CH  LEDC_CHANNEL_2

#define D_PWM_PIN  21
#define D_IN1_PIN  42
#define D_IN2_PIN  38
#define D_LEDC_CH  LEDC_CHANNEL_1

#define MOTOR_LEDC_TIMER    LEDC_TIMER_1
#define MOTOR_LEDC_SPEED    LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_FREQ     5000
#define MOTOR_LEDC_RES_BITS LEDC_TIMER_8_BIT

// 超声波传感器引脚定义
#define ULTRASONIC_TRIGGER_PIN 9  // 连接到HC-SR04的Trig引脚
#define ULTRASONIC_ECHO_PIN    12  // 连接到HC-SR04的Echo引脚
#define OBSTACLE_THRESHOLD_CM  5 // 默认障碍物检测距离(厘米)

static bool robot_is_moving = false;
// Obstacle detection parameters
#define DEFAULT_OBSTACLE_THRESHOLD_CM 5.0f
#define OBSTACLE_CHECK_INTERVAL_MS 200

static bool obstacle_detection_enabled = false;
static float obstacle_threshold_cm = DEFAULT_OBSTACLE_THRESHOLD_CM;
static TaskHandle_t obstacle_detection_task_handle = NULL;

static void set_motor(uint8_t in1_pin, uint8_t in2_pin, ledc_channel_t pwm_ch, int speed)
{
    speed = speed > 255 ? 255 : (speed < -255 ? -255 : speed);
    if (speed > 0) {
        gpio_set_level(in1_pin, 1);
        gpio_set_level(in2_pin, 0);
        ledc_set_duty(MOTOR_LEDC_SPEED, pwm_ch, speed);
        ledc_update_duty(MOTOR_LEDC_SPEED, pwm_ch);
    } else if (speed < 0) {
        gpio_set_level(in1_pin, 0);
        gpio_set_level(in2_pin, 1);
        ledc_set_duty(MOTOR_LEDC_SPEED, pwm_ch, -speed);
        ledc_update_duty(MOTOR_LEDC_SPEED, pwm_ch);
    } else {
        gpio_set_level(in1_pin, 0);
        gpio_set_level(in2_pin, 0);
        ledc_set_duty(MOTOR_LEDC_SPEED, pwm_ch, 0);
        ledc_update_duty(MOTOR_LEDC_SPEED, pwm_ch);
    }
}

// Obstacle detection task
static void obstacle_detection_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Obstacle detection task started");
    
    while (1) {
        if (obstacle_detection_enabled) {
            if (ultrasonic_obstacle_detected(obstacle_threshold_cm)) {
                ESP_LOGI(TAG, "Obstacle detected! Stopping robot");
                robot_stop();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(OBSTACLE_CHECK_INTERVAL_MS));
    }
}

esp_err_t robot_obstacle_detection_init(uint8_t trigger_pin, uint8_t echo_pin)
{
    esp_err_t ret = ultrasonic_init(trigger_pin, echo_pin);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ultrasonic sensor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 创建障碍物检测任务
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        obstacle_detection_task, 
        "obstacle_detection",
        4096,
        NULL,
        tskIDLE_PRIORITY + 1,
        &obstacle_detection_task_handle,
        1  // 跑在 Core 1
    );
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create obstacle detection task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Obstacle detection initialized with trigger=%d, echo=%d", trigger_pin, echo_pin);
    return ESP_OK;
}

void robot_obstacle_detection_enable(float threshold_cm)
{
    obstacle_threshold_cm = threshold_cm;
    obstacle_detection_enabled = true;
    ESP_LOGI(TAG, "Obstacle detection enabled with threshold %.2f cm", threshold_cm);
}

void robot_obstacle_detection_disable(void)
{
    obstacle_detection_enabled = false;
    ESP_LOGI(TAG, "Obstacle detection disabled");
}

bool robot_is_obstacle_detected(void)
{
    return ultrasonic_obstacle_detected(obstacle_threshold_cm);
}

void robot_safe_move(void (*movement_function)(void))
{
    if (!robot_is_obstacle_detected() && !is_cliff_detected()) {
        movement_function();
    } else {
        if (robot_is_obstacle_detected()) {
            ESP_LOGW(TAG, "Cannot move, obstacle detected");
        }
        if (is_cliff_detected()) {
            ESP_LOGW(TAG, "Cannot move, cliff detected");
        }
        robot_stop();
    }
}

void robot_motion_init(void)
{
    // 初始化GPIO为输出
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        // .pin_bit_mask = (1ULL << A_IN1_PIN) | (1ULL << A_IN2_PIN) | (1ULL << B_IN1_PIN) | (1ULL << B_IN2_PIN) |
        //                 (1ULL << C_IN1_PIN) | (1ULL << C_IN2_PIN) | (1ULL << D_IN1_PIN) | (1ULL << D_IN2_PIN),

        .pin_bit_mask = (1ULL << A_IN1_PIN) | (1ULL << A_IN2_PIN) |
        (1ULL << D_IN1_PIN) | (1ULL << D_IN2_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // 初始化LEDC PWM定时器
    ledc_timer_config_t motor_timer = {
        .speed_mode       = MOTOR_LEDC_SPEED,
        .timer_num        = MOTOR_LEDC_TIMER,
        .duty_resolution  = MOTOR_LEDC_RES_BITS,
        .freq_hz          = MOTOR_LEDC_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer));

    // 初始化四个通道的PWM
    ledc_channel_config_t ledc_channel[] = {
        {.gpio_num = A_PWM_PIN, .speed_mode = MOTOR_LEDC_SPEED, .channel = A_LEDC_CH, .timer_sel = MOTOR_LEDC_TIMER, .duty = 0, .hpoint = 0},
        // {.gpio_num = B_PWM_PIN, .speed_mode = MOTOR_LEDC_SPEED, .channel = B_LEDC_CH, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0},
        // {.gpio_num = C_PWM_PIN, .speed_mode = MOTOR_LEDC_SPEED, .channel = C_LEDC_CH, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0},
        {.gpio_num = D_PWM_PIN, .speed_mode = MOTOR_LEDC_SPEED, .channel = D_LEDC_CH, .timer_sel = MOTOR_LEDC_TIMER, .duty = 0, .hpoint = 0},
    };
    for (int i = 0; i < 2; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
    
    // Initialize cliff sensor
    cliff_sensor_init();
    
    // Initially robot is not moving
    robot_is_moving = false;
    cliff_detection_enable(false);

    esp_err_t ret = robot_obstacle_detection_init(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update the Ultrasonic %s", esp_err_to_name(ret));
    } else {
        // 启用障碍物检测
        robot_obstacle_detection_enable(OBSTACLE_THRESHOLD_CM);
        ESP_LOGI(TAG, "已启用障碍物检测，阈值为 %d 厘米", OBSTACLE_THRESHOLD_CM);
    }
    
    ESP_LOGI(TAG, "Robot motion system initialized");
}

void robot_move_forward(void)
{
    // Check if cliff is detected before moving
    if (obstacle_detection_enabled && robot_is_obstacle_detected()) {
        ESP_LOGW(TAG, "Cannot move forward, obstacle detected");
        robot_stop();
        return;
    }
    if (is_cliff_detected()) {
        ESP_LOGW(TAG, "Cannot move forward - cliff detected!");
        robot_stop();
        return;
    }
    
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  30);
    // set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  30);
    // set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  24);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  24);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Moving forward with cliff detection enabled");
}

void robot_move_backward(void)
{
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  -30);
    // set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  -30);
    // set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  -24);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  -24);
    
    robot_is_moving = true;
    cliff_detection_enable(false);
    ESP_LOGI(TAG, "Moving backward with cliff detection enabled");
}

void robot_turn_left(void)
{
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  -50);
    // set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  -50);
    // set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  50);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  50);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Turning left with cliff detection enabled");
}

void robot_turn_right(void)
{
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  50);
    // set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  50);
    // set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  -50);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  -50);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Turning right with cliff detection enabled");
}

void robot_spin_around(void)
{
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  30);
    // set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  30);
    // set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  -30);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  -30);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Spinning around with cliff detection enabled");
}

void robot_stop(void)
{
    // A 通道
    ledc_stop(MOTOR_LEDC_SPEED, A_LEDC_CH, 0);
    // D 通道
    ledc_stop(MOTOR_LEDC_SPEED, D_LEDC_CH, 0);

    gpio_set_level(A_IN1_PIN, 0);
    gpio_set_level(A_IN2_PIN, 0);
    gpio_set_level(D_IN1_PIN, 0);
    gpio_set_level(D_IN2_PIN, 0);

    robot_is_moving = false;
    cliff_detection_enable(false);
    ESP_LOGI(TAG, "Robot stopped, cliff detection disabled");
}

bool robot_is_in_motion(void)
{
    return robot_is_moving;
}