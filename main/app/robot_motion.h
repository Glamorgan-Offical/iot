#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include <stdint.h>
void robot_motion_init(void);  
void robot_move_forward(void);
void robot_move_backward(void);
void robot_turn_left(void);
void robot_turn_right(void);
void robot_spin_around(void);
void robot_stop(void);

#endif // ROBOT_MOTION_H
