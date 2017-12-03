#include <stdint.h>
#include "stepper.h"
#include "stepper_control.h"
#include "gpio.h"
#include "serial.h"
#include "string.h"

unsigned char cal = 0; // calibration flag

// mapping of squares to mm for stepper driver
#define N 10
grid_t grid[N][N] = { // y,x in mm to each square (from origin)
//   x -->
// y
// |
// V
/*		0         1			2		   3		  4			 5			6		   7		  8			 9		  */
/* 0 */ {{0,0},   {0,68},    {0,133},   {0,196},   {0,259},   {0,322},   {0,386},   {0,450},   {0,513},   {0,577}},
/* 1 */ {{65,0},  {65,68},  {65,133},  {65,197},  {65,260},  {65,323},  {65,386},  {65,449},  {65,512},  {65,576}},
/* 2 */	{{130,0}, {130,64}, {130,128}, {130,192}, {130,256}, {130,320}, {130,384}, {130,448}, {130,512}, {130,576}},
/* 3 */	{{192,0}, {192,64}, {192,128}, {192,192}, {192,256}, {192,320}, {192,384}, {192,448}, {192,512}, {192,576}},
/* 4 */	{{254,0}, {254,64}, {254,128}, {254,192}, {254,256}, {254,320}, {254,384}, {254,448}, {254,512}, {254,576}},
/* 5 */	{{317,0}, {315,64}, {320,128}, {320,192}, {320,256}, {320,320}, {320,384}, {320,448}, {320,512}, {320,576}},
/* 6 */	{{383,0}, {378,64}, {384,128}, {384,192}, {384,256}, {384,320}, {384,384}, {384,448}, {384,512}, {384,576}},
/* 7 */	{{449,0}, {443,64}, {448,128}, {448,192}, {448,256}, {448,320}, {448,384}, {448,448}, {448,512}, {448,576}},
/* 8 */	{{512,0}, {506,64}, {512,128}, {512,192}, {512,256}, {512,320}, {512,384}, {512,448}, {512,512}, {512,576}},
/* 9 */	{{572,0}, {572,64}, {576,128}, {576,192}, {576,256}, {576,320}, {576,384}, {576,448}, {576,512}, {576,576}},
};
uint8_t w_offset = 0; // graveyard offset, in square or on line
uint8_t b_offset = 0;

// sets slot.x and slot.y to where the captured piece should go in graveyard
// returns 0 if slot is available, -1 if there are no more slots
int get_current_graveyard_slot(grid_t *slot, char color) {
	uint8_t *offset = color == 'w' ? &w_offset : &b_offset;
	
	if (*offset < (N*2)-1) {
		grid_t *graveyard_ptr = color == 'w' ? &(grid[0][1])+(N*(w_offset/2)) : &(grid[0][N-2])+(N*(b_offset/2));
		if (*offset % 2 == 0) { // in square
			slot->x = graveyard_ptr->x;
			slot->y = graveyard_ptr->y;
		} else { // on line
			slot->x = graveyard_ptr->x + (((graveyard_ptr+N)->x - graveyard_ptr->x)/2);
			slot->y = graveyard_ptr->y + (((graveyard_ptr+N)->y - graveyard_ptr->y)/2);
		}
		*offset += 1;
		LOG_TRACE("graveyard slot: (%d,%d), w_offset: %d, b_offset: %d", slot->x, slot->y, w_offset, b_offset);
		return 0;
	}
	return -1; // no more slots
}

int magnet_on(void) {
	LOG_TRACE("Turning magnet on");
	MAGNET_ON;
	HAL_Delay(500);
	return 0;
} 

int magnet_off(void) {
	LOG_TRACE("Turning magnet off");
	int x = get_pos(X);
	int y = get_pos(Y);
	LOG_TRACE("magnet position: (%d,%d)", x, y);
	HAL_Delay(500);
	MAGNET_OFF; 
	return 0;
}

int move_done(void) {
	magnet_off();
	SEND_CMD_P(CMD_STATUS, "%d", STATUS_OKAY);
	return 0;
}

int set_origin(void) {
	step_reset();
	w_offset = b_offset = 0;
	int x = get_pos(X);
	int y = get_pos(Y);
	cal = 0;
	return 0;
}

void debug_squares(void) {
	int i, j;
	for (i=0; i<N; i++) {
		for (j=0; j<N; j++) {
			debug_move(i, j);
		}
	}
}

void debug_move(int16_t x, int16_t y) {
	LOG_TRACE("debug_move");
	LOG_TRACE("get_pos(X)=%d, get_pos(Y)=%d", get_pos(X), get_pos(Y));
	LOG_TRACE("grid[y][x].x=%d, grid[y][x].y=%d", grid[y][x].x, grid[y][x].y);
	int16_t dest_x = grid[y][x].x - get_pos(X);
	int16_t dest_y = grid[y][x].y - get_pos(Y);
	LOG_TRACE("dest_x=%d, dest_y=%d", dest_x, dest_y);
	step_mm_blocking(dest_x, 0); // goto src
	step_mm_blocking(0, dest_y); // goto src
}

void get_promoted_piece(grid_t *slot, char color) {
	if (color == 'w') {
		slot->x = grid[0][0].x;
		slot->y = grid[0][0].y;
	} else { // black
		slot->x = grid[9][9].x;
		slot->y = grid[9][9].y;
	}
}

/*
 *  move chess piece at (x,y) to (dest_x,dest_y)
 *	x, y, are in grid indexes (0 to N-1) and 
 *	dest_x, dest_y are in mm
 *
*	Note: mainly for moving to graveyard
 */
void move_piece_to_mm(uint8_t x, uint8_t y, int16_t dest_x, int16_t dest_y) {
	 LOG_TRACE("move_piece_to_mm: x=%d, y=%d, dest_x=%d, dest_y=%d", x, y, dest_x, dest_y);
	 // goto src
	 int16_t x_align = DIFF_MM(grid[y][x].x, get_pos(X));
	 int16_t y_align = DIFF_MM(grid[y][x].y, get_pos(Y));
	 // move to dst
	 int16_t dx = DIFF_MM(dest_x, grid[y][x].x);
	 int16_t dy = DIFF_MM(dest_y, grid[y][x].y);
	
	 LOG_TRACE("x_align=%d, y_align=%d, dx=%d, dy=%d", x_align, y_align, dx, dy);
	 int16_t x_off, y_off;
     SET_OFFSET(x_off, y_off);
	 
	 LOG_TRACE("x_off=%d, y_off=%d", x_off, y_off);
	 TAXI_CAB_MOVE(x_align,y_align,x_off,y_off,dx,dy);
}

/*
 *  move chess piece at (x,y) to (dest_x,dest_y)
 *	x, y, dest_x, dest_y are in grid indexes (0 to N-1)
 */
void move_piece(int16_t x, int16_t y, int16_t dest_x, int16_t dest_y) {
	 LOG_TRACE("move_piece: x=%d, y=%d, dest_x=%d, dest_y=%d", x, y, dest_x, dest_y);
	 // goto src
	 int16_t x_align = DIFF_MM(grid[y][x].x, get_pos(X));
	 int16_t y_align = DIFF_MM(grid[y][x].y, get_pos(Y));
	 // move to dst
	 int16_t dx = DIFF_MM(grid[dest_y][dest_x].x, grid[y][x].x);
	 int16_t dy = DIFF_MM(grid[dest_y][dest_x].y, grid[y][x].y);
	
	 LOG_TRACE("x_align=%d, y_align=%d, dx=%d, dy=%d", x_align, y_align, dx, dy);
	 int16_t x_off, y_off;
     SET_OFFSET(x_off, y_off);
	 
	 LOG_TRACE("x_off=%d, y_off=%d", x_off, y_off);
	 TAXI_CAB_MOVE(x_align,y_align,x_off,y_off,dx,dy);
}

/*
 *  Universal Chess Interface (UCI) move
 *  move is in the long algebraic form
 *	i.e.: [type][src_file][src_rank][x]?[dst_file][dst_rank][promotion_type]?
 *	example:	e2e4 (pawn do not indicate piece type)
 *				Nd3d7 (Night at d3 moves to d7)
 *				e7e8q (pawn on e7 promotion to queen on e8)
 *				Rd3xd7 (rook on d3 captures piece on d7)
 *	valid files: a through h
 *	valid ranks: 1 through 8
 *  move types by length:
 *		4 = pawn_move
 *		5 = {non_pawn_move, pawn_capture, pawn_promotion}
 *		6 = non_pawn_capture
 */
int uci_move(const char *move) {
	
	// TODO: add a lot more error checking
	uint8_t src_x, src_y, dst_x, dst_y;
	
	if(strlen(move) == 4) {
		SET_COORDS(src_x, src_y, move);
		SET_COORDS(dst_x, dst_y, move+2);
	} else {
		SEND_CMD_P(CMD_STATUS, "%d", STATUS_FAIL);
		return -1;
	}

	if (COORD_INVALID(src_x) || COORD_INVALID(src_y) || COORD_INVALID(dst_x) || COORD_INVALID(dst_y)) {
		SEND_CMD_P(CMD_STATUS, "%d", STATUS_FAIL);
		return -1;
	}
	
	move_piece(src_x, src_y, dst_x, dst_y);
	return 0;
}

unsigned char calibrating(void) {
	return cal;
}

int calibrate(void) {
	// step until hit calibration switches
	cal = 1;
	uint32_t timeout = HAL_GetTick() + CALIBRATION_TIMEOUT;
	unsigned char x_done = 0;
	unsigned char y_done = 0;
	int ret = 0;
	step_mm(X, -2000);
	step_mm(Y, -2000);
	
	LOG_TRACE("beginning calibration...");
	while (1) {
		if (x_done && y_done) {
			break;
		} else if (GPIOC->IDR & (1 << X_CAL)) {
			stop_axis(X);
			x_done = 1;
		} else if (GPIOC->IDR & (1 << Y_CAL)) {
			stop_axis(Y);
			y_done = 1;
		} else if (HAL_GetTick() > timeout) { // timeout
			stop_stepping();
			ret = -1;
			break;
		}
    }
	step_mm_blocking(ORIGIN_X, ORIGIN_Y);
	set_origin();
	LOG_TRACE("done calibrating");
	return ret;
}

/*
 *	for user button (kill switch)
 */
void HAL_SYSTICK_Callback(void) {
	static uint8_t x_debouncer = 0;
	static uint8_t y_debouncer = 0;
	
	if (!calibrating()) {
		if (axis_stepping(X)) {
			x_debouncer = (x_debouncer << 1);
			if (GPIOC->IDR & (1 << X_CAL)) {
				x_debouncer |= 0x1;
			}
			if (x_debouncer == 0x7F) {
				x_debouncer = 0;
				stop_stepping();
				step_mm(X, 2);
			}
			if (get_pos(X) > UPPER_LIMIT) {
				stop_stepping();
				step_mm(X, -2);
			}
		}
		
		if (axis_stepping(Y)) {
			y_debouncer = (y_debouncer << 1);
			if (GPIOC->IDR & (1 << Y_CAL)) {
				y_debouncer |= 0x1;
			}
			if (y_debouncer == 0x7F) {
				y_debouncer = 0;
				stop_stepping();
				step_mm(Y, 2);
			}
			if (get_pos(Y) > UPPER_LIMIT) {
				stop_stepping();
				step_mm(Y, -2);
			}
		}
	}
    
}
