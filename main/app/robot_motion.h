#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

void robot_motion_init(void);  
void robot_move_forward(void);
void robot_move_forward_timed(int time_ms);
void robot_move_backward(void);
void robot_move_backward_timed(int time_ms);
void robot_turn_left(void);
void robot_turn_right(void);
void robot_turn_left_timed(int time_ms);
void robot_turn_right_timed(int time_ms);
void robot_spin_around(void);
void robot_spin_around_timed(int time_ms);
void robot_stop(void);
void robot_pause(void);
bool robot_is_in_motion(void);
esp_err_t robot_obstacle_detection_init(uint8_t trigger_pin, uint8_t echo_pin);
void robot_obstacle_detection_enable(float threshold_cm);
void robot_obstacle_detection_disable(void);
bool robot_is_obstacle_detected(void);
void robot_safe_move(void (*movement_function)(void));
void robot_start_walk_around(void);
void robot_stop_walk_around(void);
bool robot_is_walk_around_active(void);

#endif // ROBOT_MOTION_H
