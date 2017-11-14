#ifndef __STEPPER_CTRL_H_
#define __STEPPER_CTRL_H_

#include "queue.h"

// chess board constants
#define SQUARE_WIDTH 62 // mm
#define SQUARE_HALF_WIDTH 31 // mm
#define X_WIDTH_ERR 0 // mm
#define Y_WIDTH_ERR 2 // mm
#define LOWER_LIMIT 0-(SQUARE_WIDTH+SQUARE_HALF_WIDTH)// mm
#define UPPER_LIMIT 600 // mm
#define ORIGIN_X 41 // mm
#define ORIGIN_Y 25 // mm
#define CALIBRATION_TIMEOUT 10000 // ms

// macros
#define SQUARES_TO_MM(squares)     (SQUARE_WIDTH*(squares))
#define HALF_SQUARES_TO_MM(halves) (SQUARE_HALF_WIDTH*(halves))

// magnet
#define MAGNET_PIN 0 // GPIOC
#define MAGNET_OFF (GPIOC->ODR &= ~(1 << MAGNET_PIN))
#define MAGNET_ON (GPIOC->ODR |=  (1 << MAGNET_PIN))

// movement
#define SET_COORDS(x,y,uci) do {(x)=(uci)[0]-'a'+1; (y)=(uci)[1]-'1'+1;} while(0)
#define COORD_INVALID(c) ((c) < 1 || (c) > 8)
#define MOVE_PIECE_TO_GRAVEYARD(x,y,g,c) do { \
	if ((c) == 'w' || (c) == 'b') { \
		get_current_graveyard_slot(&(g), (c));   \
		move_piece((x), (y), (g).x, (g).y); \
	} \
} while(0)

typedef struct {
	int16_t y;
	int16_t x;
} grid_t;

// func definitions
int get_current_graveyard_slot(grid_t *slot, char color);
void step_control_init(void);
void step_squares(int axis, int n);
void debug_move(int16_t x, int16_t y);
void move_piece(int16_t x, int16_t y, int16_t dest_x, int16_t dest_y);
int  uci_move(const char *move);
void add_to_queue(int16_t x, int16_t y);
void add_to_queue_d(int16_t x, int16_t y, done_func d);
void empty_queue(void);
int magnet_on(void);
int magnet_off(void);
int move_done(void);
int set_origin(void);
unsigned char calibrating(void);
int calibrate(void);

#endif /* __STEPPER_CTRL_H_ */
