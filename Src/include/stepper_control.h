#ifndef __STEPPER_CTRL_H_
#define __STEPPER_CTRL_H_

#include "queue.h"

// chess board constants
#define SQUARE_WIDTH 64 // mm
#define SQUARE_HALF_WIDTH 32 // mm
#define UPPER_LIMIT SQUARE_WIDTH*7 // mm

// macros
#define SQUARES_TO_MM(squares)     (SQUARE_WIDTH*squares)
#define HALF_SQUARES_TO_MM(halves) (SQUARE_HALF_WIDTH*halves)

// magnet
#define MAGNET_PIN 0 // GPIOC
#define MAGNET_OFF (GPIOC->ODR &= ~(1 << MAGNET_PIN))
#define MAGNET_ON (GPIOC->ODR |=  (1 << MAGNET_PIN))

// func definitions
void step_control_init(void);
void step_squares(int axis, int n);
void move_piece(int x, int y, int dest_x, int dest_y);
int  uci_move(const char *move);
void add_to_queue(int x, int y);
void add_to_queue_d(int x, int y, done_func d);
void empty_queue(void);
int magnet_on(void);
int magnet_off(void);
int move_done(void);
int set_origin(void);
unsigned char calibrating(void);
int calibrate(void);

#endif /* __STEPPER_CTRL_H_ */
