#include "robot_motion.h"
#include "cliff_sensor.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ultrasonic_sensor.h"
#include "driver/ledc.h"  
#include <stdlib.h>  
#include "app_ui_ctrl.h" 

static const char *TAG = "robot_motion";

// Four motor pins and PWM channel definitions
#define A_PWM_PIN  10
#define A_IN1_PIN  14
#define A_IN2_PIN  11
#define A_LEDC_CH  LEDC_CHANNEL_0

#define B_PWM_PIN  41
#define B_IN1_PIN  44
#define B_IN2_PIN  43
#define B_LEDC_CH  LEDC_CHANNEL_3

#define C_PWM_PIN  21
#define C_IN1_PIN  42
#define C_IN2_PIN  19
#define C_LEDC_CH  LEDC_CHANNEL_4

#define D_PWM_PIN  38
#define D_IN1_PIN  20
#define D_IN2_PIN  39
#define D_LEDC_CH  LEDC_CHANNEL_2

#define MOTOR_LEDC_TIMER    LEDC_TIMER_1
#define MOTOR_LEDC_SPEED    LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_FREQ     5000
#define MOTOR_LEDC_RES_BITS LEDC_TIMER_8_BIT

// Ultrasonic sensor pin definitions
#define ULTRASONIC_TRIGGER_PIN 9  
#define ULTRASONIC_ECHO_PIN    12  
#define OBSTACLE_THRESHOLD_CM  10 

static bool robot_is_moving = false;
// Obstacle detection parameters
#define DEFAULT_OBSTACLE_THRESHOLD_CM 10.0f
#define OBSTACLE_CHECK_INTERVAL_MS 200

// Walk Around feature parameters
#define WALK_AROUND_CHECK_INTERVAL_MS 300
#define TURN_DURATION_MS 1300
#define BACKWARD_DURATION_MS 1800

static bool obstacle_detection_enabled = false;
static float obstacle_threshold_cm = DEFAULT_OBSTACLE_THRESHOLD_CM;
static TaskHandle_t obstacle_detection_task_handle = NULL;
static TaskHandle_t walk_around_task_handle = NULL;  
static bool walk_around_active = false;  

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
                robot_pause();
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
    
    // Create obstacle detection task
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        obstacle_detection_task, 
        "obstacle_detection",
        4096,
        NULL,
        tskIDLE_PRIORITY + 1,
        &obstacle_detection_task_handle,
        1  // Run on Core 1
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
        robot_pause();
    }
}

void robot_motion_init(void)
{
    // Initialize GPIO as output
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << A_IN1_PIN) | (1ULL << A_IN2_PIN) | (1ULL << B_IN1_PIN) | (1ULL << B_IN2_PIN) |
                        (1ULL << C_IN1_PIN) | (1ULL << C_IN2_PIN) | (1ULL << D_IN1_PIN) | (1ULL << D_IN2_PIN),

        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Initialize LEDC PWM timer
    ledc_timer_config_t motor_timer = {
        .speed_mode       = MOTOR_LEDC_SPEED,
        .timer_num        = MOTOR_LEDC_TIMER,
        .duty_resolution  = MOTOR_LEDC_RES_BITS,
        .freq_hz          = MOTOR_LEDC_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer));

    // Initialize PWM for four channels
    ledc_channel_config_t ledc_channel[] = {
        {.gpio_num = A_PWM_PIN, .speed_mode = MOTOR_LEDC_SPEED, .channel = A_LEDC_CH, .timer_sel = MOTOR_LEDC_TIMER, .duty = 0, .hpoint = 0},
        {.gpio_num = B_PWM_PIN, .speed_mode = MOTOR_LEDC_SPEED, .channel = B_LEDC_CH, .timer_sel = MOTOR_LEDC_TIMER, .duty = 0, .hpoint = 0},
        {.gpio_num = C_PWM_PIN, .speed_mode = MOTOR_LEDC_SPEED, .channel = C_LEDC_CH, .timer_sel = MOTOR_LEDC_TIMER, .duty = 0, .hpoint = 0},
        {.gpio_num = D_PWM_PIN, .speed_mode = MOTOR_LEDC_SPEED, .channel = D_LEDC_CH, .timer_sel = MOTOR_LEDC_TIMER, .duty = 0, .hpoint = 0},
    };
    for (int i = 0; i < 4; i++) {
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
        // Enable obstacle detection
        robot_obstacle_detection_enable(OBSTACLE_THRESHOLD_CM);
        ESP_LOGI(TAG, "Obstacle detection enabled with threshold %d cm", OBSTACLE_THRESHOLD_CM);
    }
    
    ESP_LOGI(TAG, "Robot motion system initialized");
}

void robot_move_forward(void)
{
    // Check if cliff is detected before moving
    if (obstacle_detection_enabled && robot_is_obstacle_detected()) {
        ESP_LOGW(TAG, "Cannot move forward, obstacle detected");
        robot_pause();
        return;
    }
    if (is_cliff_detected()) {
        ESP_LOGW(TAG, "Cannot move forward - cliff detected!");
        robot_stop();
        return;
    }
    
    // Use MOVING panel to indicate movement status
    ui_ctrl_show_panel(UI_CTRL_PANEL_MOVING, 0);
    
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  19);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  19);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  16);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  16);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Moving forward with cliff detection enabled");
}

void robot_move_backward(void)
{

    ui_ctrl_show_panel(UI_CTRL_PANEL_MOVING, 0);
    
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  -19);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  -19);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  -16);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  -16);
    
    robot_is_moving = true;
    cliff_detection_enable(false);
    ESP_LOGI(TAG, "Moving backward with cliff detection enabled");
}

void robot_turn_left(void)
{
    // Use MOVING panel to indicate movement status
    ui_ctrl_show_panel(UI_CTRL_PANEL_MOVING, 0);
    
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  -35);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  -35);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  60);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  60);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Turning left with cliff detection enabled");
}

void robot_turn_right(void)
{
    // Use MOVING panel to indicate movement status
    ui_ctrl_show_panel(UI_CTRL_PANEL_MOVING, 0);
    
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  60);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  60);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  -35);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  -35);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Turning right with cliff detection enabled");
}

// Left turn function with time parameter
void robot_turn_left_timed(int time_ms)
{
    // Left turn function with time parameter
    bool prev_enabled = obstacle_detection_enabled;       // Save original state
    robot_obstacle_detection_disable();                   // Temporarily disable
    
    robot_turn_left();                                    // Start turning
    vTaskDelay(pdMS_TO_TICKS(time_ms));                   // Continue for time_ms
    robot_pause();                                        // Brake
    
    if (prev_enabled) {                                   // Restore if needed
        robot_obstacle_detection_enable(obstacle_threshold_cm);
    }
}

// Right turn function with time parameter
void robot_turn_right_timed(int time_ms)
{
    bool prev_enabled = obstacle_detection_enabled;
    robot_obstacle_detection_disable();

    robot_turn_right();
    vTaskDelay(pdMS_TO_TICKS(time_ms));
    robot_pause();

    if (prev_enabled) {
        robot_obstacle_detection_enable(obstacle_threshold_cm);
    }
}

void robot_spin_around(void)
{
    // Use MOVING panel to indicate movement status
    ui_ctrl_show_panel(UI_CTRL_PANEL_MOVING, 0);
    
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  60);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  60);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  -35);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  -35);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    ESP_LOGI(TAG, "Spinning around with cliff detection enabled");
}

// Spin function with time parameter
void robot_spin_around_timed(int time_ms)
{
    // Spin
    ui_ctrl_show_panel(UI_CTRL_PANEL_MOVING, 0);
    
    // Delay for specified time
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,   60);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,   60);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  -35);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  -35);
    
    robot_is_moving = true;
    cliff_detection_enable(true);
    
    // Delay for specified time
    vTaskDelay(pdMS_TO_TICKS(time_ms));
    robot_pause();
    
    ESP_LOGI(TAG, "Spun around for %d ms", time_ms);
}

// Pause robot movement (does not terminate walk around task)
void robot_pause(void)
{
    // Keep UI in awake state
    // ui_ctrl_show_panel(UI_CTRL_PANEL_MOVING, 0);
    
    ledc_set_duty(MOTOR_LEDC_SPEED, A_LEDC_CH, 0);
    ledc_update_duty(MOTOR_LEDC_SPEED, A_LEDC_CH);

    ledc_set_duty(MOTOR_LEDC_SPEED, B_LEDC_CH, 0);
    ledc_update_duty(MOTOR_LEDC_SPEED, B_LEDC_CH);

    ledc_set_duty(MOTOR_LEDC_SPEED, C_LEDC_CH, 0);
    ledc_update_duty(MOTOR_LEDC_SPEED, C_LEDC_CH);

    ledc_set_duty(MOTOR_LEDC_SPEED, D_LEDC_CH, 0);
    ledc_update_duty(MOTOR_LEDC_SPEED, D_LEDC_CH);

    /* ----------- Disable motor direction ------------ */
    gpio_set_level(A_IN1_PIN, 0);
    gpio_set_level(A_IN2_PIN, 0);

    gpio_set_level(B_IN1_PIN, 0);
    gpio_set_level(B_IN2_PIN, 0);
    
    gpio_set_level(C_IN1_PIN, 0);
    gpio_set_level(C_IN2_PIN, 0);
    
    gpio_set_level(D_IN1_PIN, 0);
    gpio_set_level(D_IN2_PIN, 0);

   
    robot_is_moving = false;
    
    ESP_LOGI(TAG, "Robot paused (temporary stop)");
}

void robot_stop(void)
{
    // If currently in walk around mode, stop the walk around task first
    if (walk_around_active) {
        robot_stop_walk_around();
        ESP_LOGI(TAG, "Walk around mode stopped by robot_stop");
    }

    ledc_set_duty(MOTOR_LEDC_SPEED, A_LEDC_CH, 0);
    ledc_update_duty(MOTOR_LEDC_SPEED, A_LEDC_CH);

    ledc_set_duty(MOTOR_LEDC_SPEED, B_LEDC_CH, 0);
    ledc_update_duty(MOTOR_LEDC_SPEED, B_LEDC_CH);

    ledc_set_duty(MOTOR_LEDC_SPEED, C_LEDC_CH, 0);
    ledc_update_duty(MOTOR_LEDC_SPEED, C_LEDC_CH);

    ledc_set_duty(MOTOR_LEDC_SPEED, D_LEDC_CH, 0);
    ledc_update_duty(MOTOR_LEDC_SPEED, D_LEDC_CH);

   
    gpio_set_level(A_IN1_PIN, 0);
    gpio_set_level(A_IN2_PIN, 0);

    gpio_set_level(B_IN1_PIN, 0);
    gpio_set_level(B_IN2_PIN, 0);
    
    gpio_set_level(C_IN1_PIN, 0);
    gpio_set_level(C_IN2_PIN, 0);
    
    gpio_set_level(D_IN1_PIN, 0);
    gpio_set_level(D_IN2_PIN, 0);

    robot_is_moving = false;
    cliff_detection_enable(false);
    
    ui_ctrl_show_panel(UI_CTRL_PANEL_SLEEP, 3000);
    
    ESP_LOGI(TAG, "Robot stopped, cliff detection disabled");
}

bool robot_is_in_motion(void)
{
    return robot_is_moving;
}

// Robot autonomous walking task
static void walk_around_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Walk around task started");
    
    // Initial forward movement
    robot_move_forward();
    
    // Counter for periodically updating UI
    int ui_update_counter = 0;
    
    while (walk_around_active) {
        // Refresh UI display periodically to ensure UI stays awake during activity
        if (++ui_update_counter >= 10) { 
            ui_ctrl_show_panel(UI_CTRL_PANEL_MOVING, 0);
            ui_update_counter = 0;
        }
        
        // Check for obstacles
        if (robot_is_obstacle_detected()) {
            ESP_LOGI(TAG, "Obstacle detected during walk around");
            
            // Stop - use pause rather than complete stop
            robot_pause();
            vTaskDelay(pdMS_TO_TICKS(500));
            
            // Randomly choose left or right turn, using timed turn function
            if (rand() % 2 == 0) {
                ESP_LOGI(TAG, "Random turn: left");
                robot_turn_left_timed(TURN_DURATION_MS);
            } else {
                ESP_LOGI(TAG, "Random turn: right");
                robot_turn_right_timed(TURN_DURATION_MS);
            }
            
            robot_move_forward();
        }
        
        // Check for cliffs
        if (is_cliff_detected()) {
            ESP_LOGI(TAG, "Cliff detected during walk around");
            
            // Pause first
            robot_pause();
            vTaskDelay(pdMS_TO_TICKS(200));
            
            // Move backward
            robot_move_backward();
            vTaskDelay(pdMS_TO_TICKS(BACKWARD_DURATION_MS));
            
            // Pause
            robot_pause();
            vTaskDelay(pdMS_TO_TICKS(200));
            
            // Randomly choose left or right turn, using timed turn function
            if (rand() % 2 == 0) {
                ESP_LOGI(TAG, "Random turn: left");
                robot_turn_left_timed(TURN_DURATION_MS + 800);  // Larger turn angle when cliff is detected
            } else {
                ESP_LOGI(TAG, "Random turn: right");
                robot_turn_right_timed(TURN_DURATION_MS + 800);  // Larger turn angle when cliff is detected
            }
            
            // Continue forward
            robot_move_forward();
        }
        
        // Check interval
        vTaskDelay(pdMS_TO_TICKS(WALK_AROUND_CHECK_INTERVAL_MS));
    }
    
    // Task end, stop robot
    robot_pause(); // Use pause instead of stop here, because when task ends by itself, it doesn't need to stop itself again
    ESP_LOGI(TAG, "Walk around task stopped");
    
    // Delete self task
    walk_around_task_handle = NULL;
    vTaskDelete(NULL);
}

// Start autonomous walking mode
void robot_start_walk_around(void)
{
    ui_ctrl_show_panel(UI_CTRL_PANEL_MOVING, 0);
    
    // If already in walk around mode, stop and restart
    if (walk_around_active) {
        robot_stop_walk_around();
        vTaskDelay(pdMS_TO_TICKS(300));  
    }
    srand(xTaskGetTickCount());
    
    if (!obstacle_detection_enabled) {
        robot_obstacle_detection_enable(obstacle_threshold_cm);
    }
    
    cliff_detection_enable(true);
    
    walk_around_active = true;
    
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        walk_around_task, 
        "walk_around_task",
        4096,
        NULL,
        tskIDLE_PRIORITY + 1,
        &walk_around_task_handle,
        1  
    );
    
    if (xReturned != pdPASS) {
        ESP_LOGE(TAG, "Failed to create walk around task");
        walk_around_active = false;
        return;
    }
    
    ESP_LOGI(TAG, "Walk around mode started");
}

// Stop autonomous walking mode
void robot_stop_walk_around(void)
{
    if (!walk_around_active) {
        ESP_LOGW(TAG, "Walk around not active");
        return;
    }
    
    // Set flag to let the task end by itself
    walk_around_active = false;

    if (walk_around_task_handle != NULL) {

        for (int i = 0; i < 10 && walk_around_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // If task still not ended, force delete
        if (walk_around_task_handle != NULL) {
            TaskHandle_t temp_handle = walk_around_task_handle;
            walk_around_task_handle = NULL;
            vTaskDelete(temp_handle);
        }
    }
    

    cliff_detection_enable(false); 
    
    ESP_LOGI(TAG, "Walk around mode stopped");
}

// Check if in autonomous walking mode
bool robot_is_walk_around_active(void)
{
    return walk_around_active;
}

// Forward function with time parameter
void robot_move_forward_timed(int time_ms)
{
    ui_ctrl_show_panel(UI_CTRL_PANEL_MOVING, 0);
    
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  24);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  24);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  20);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  20);
    
    robot_is_moving = true;
    cliff_detection_enable(true);

    vTaskDelay(pdMS_TO_TICKS(time_ms));
    
    // Stop automatically after the specified time
    robot_pause();
    
    ESP_LOGI(TAG, "Moved forward for %d ms", time_ms);
}

// Backward function with time parameter
void robot_move_backward_timed(int time_ms)
{
    ui_ctrl_show_panel(UI_CTRL_PANEL_MOVING, 0);
    
    set_motor(A_IN1_PIN, A_IN2_PIN, A_LEDC_CH,  -24);
    set_motor(B_IN1_PIN, B_IN2_PIN, B_LEDC_CH,  -24);
    set_motor(C_IN1_PIN, C_IN2_PIN, C_LEDC_CH,  -20);
    set_motor(D_IN1_PIN, D_IN2_PIN, D_LEDC_CH,  -20);
    
    robot_is_moving = true;
    cliff_detection_enable(false);
    
    // Delay for specified time
    vTaskDelay(pdMS_TO_TICKS(time_ms));
    
    // Stop automatically after the specified time
    robot_pause();
    
    ESP_LOGI(TAG, "Moved backward for %d ms", time_ms);
}