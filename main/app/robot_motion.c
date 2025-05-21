#include "robot_motion.h"
#include "cliff_sensor.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"


static const char *TAG = "robot_motion";

// 四路电机引脚和PWM通道定义
#define A_PWM_PIN  10
#define A_IN1_PIN  14
#define A_IN2_PIN  11
#define A_LEDC_CH  LEDC_CHANNEL_0

#define B_PWM_PIN  13
#define B_IN1_PIN  9
#define B_IN2_PIN  12
#define B_LEDC_CH  LEDC_CHANNEL_3

#define C_PWM_PIN  42
#define C_IN1_PIN  20
#define C_IN2_PIN  39
#define C_LEDC_CH  LEDC_CHANNEL_2

#define D_PWM_PIN  21
#define D_IN1_PIN  19
#define D_IN2_PIN  38
#define D_LEDC_CH  LEDC_CHANNEL_1

#define LEDC_FREQ     5000
#define LEDC_RES_BITS LEDC_TIMER_8_BIT

static bool robot_is_moving = false;

static void set_motor(uint8_t in1_pin, uint8_t in2_pin, ledc_channel_t pwm_ch, int speed)
{
    speed = speed > 255 ? 255 : (speed < -255 ? -255 : speed);
    if (speed > 0) {
        gpio_set_level(in1_pin, 1);
        gpio_set_level(in2_pin, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_ch, speed);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_ch);
    } else if (speed < 0) {
        gpio_set_level(in1_pin, 0);
        gpio_set_level(in2_pin, 1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_ch, -speed);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_ch);
    } else {
        gpio_set_level(in1_pin, 0);
        gpio_set_level(in2_pin, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_ch, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_ch);
    }
}

void robot_motion_init(void)
{
    // 初始化GPIO为输出
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << A_IN1_PIN) | (1ULL << A_IN2_PIN) | (1ULL << B_IN1_PIN) | (1ULL << B_IN2_PIN) |
                        (1ULL << C_IN1_PIN) | (1ULL << C_IN2_PIN) | (1ULL << D_IN1_PIN) | (1ULL << D_IN2_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // 初始化LEDC PWM定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_RES_BITS,
        .freq_hz          = LEDC_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // 初始化四个通道的PWM
    ledc_channel_config_t ledc_channel[] = {
        {.gpio_num = A_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = A_LEDC_CH, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0},
        {.gpio_num = B_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = B_LEDC_CH, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0},
        {.gpio_num = C_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = C_LEDC_CH, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0},
        {.gpio_num = D_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = D_LEDC_CH, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0},
    };
    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
    
    // Initialize cliff sensor
    cliff_sensor_init();
    
    // Initially robot is not moving
    robot_is_moving = false;
    cliff_detection_enable(false);
    
    ESP_LOGI(TAG, "Robot motion system initialized");
}

void robot_move_forward(void)
{
    // Check if cliff is detected before moving
    if (is_cliff_detected()) {
        ESP_LOGW(TAG, "Cannot move forward - cliff detected!");
        return;
    }
    
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  30);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  30);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  24);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  24);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Moving forward with cliff detection enabled");
}

void robot_move_backward(void)
{
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  -100);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  -100);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  -80);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  -80);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Moving backward with cliff detection enabled");
}

void robot_turn_left(void)
{
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  -50);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  -50);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  50);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  50);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Turning left with cliff detection enabled");
}

void robot_turn_right(void)
{
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  50);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  50);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  -50);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  -50);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Turning right with cliff detection enabled");
}

void robot_spin_around(void)
{
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  100);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  100);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  -100);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  -100);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Spinning around with cliff detection enabled");
}

void robot_stop(void)
{
    cliff_detection_enable(false);

    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH, 0);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH, 0);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH, 0);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH, 0);
    
    robot_is_moving = false;
    cliff_detection_enable(false);
    ESP_LOGI(TAG, "Robot stopped, cliff detection disabled");
}

bool robot_is_in_motion(void)
{
    return robot_is_moving;
}