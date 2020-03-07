#ifndef _MTN_LIBRARY_H
#define _MTN_LIBRARY_H

typedef enum {left_foot,right_foot} foot_t;

void mtn_lib_init(void);
void mtn_lib_stop_mtn(void);
void mtn_lib_set_start_foot(foot_t foot);

uint8_t walk_forward(void);
uint8_t walk_backward(void);
uint8_t turn_left(void);
uint8_t turn_right(void);
uint8_t walk_left(void);
uint8_t walk_right(void);
uint8_t walk_forward_turn_left(void);
uint8_t walk_forward_turn_right(void);
uint8_t walk_backward_turn_left(void);
uint8_t walk_backward_turn_right(void);
uint8_t walk_forward_left(void);
uint8_t walk_forward_right(void);
uint8_t walk_backward_left(void);
uint8_t walk_backward_right(void);

uint8_t fast_walk_forward(void);
uint8_t fast_walk_backward(void);
uint8_t fast_turn_left(void);
uint8_t fast_turn_right(void);
uint8_t fast_walk_left(void);
uint8_t fast_walk_right(void);
uint8_t fast_walk_forward_turn_left(void);
uint8_t fast_walk_forward_turn_right(void);
uint8_t fast_walk_backward_turn_left(void);
uint8_t fast_walk_backward_turn_right(void);

#endif
