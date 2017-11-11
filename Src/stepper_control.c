#include "stepper.h"
#include "stepper_control.h"
#include "queue.h"
#include "gpio.h"
#include "serial.h"
#include "string.h"

tuple_queue_t steps; // in mm (makes stepping half squares more accurate)
tuple_t current = {0,0,NULL}; // current step tuple
unsigned char cal = 0; // calibration flag
unsigned long systime = 0;

// mapping of squares to mm for stepper driver
grid_t pos = {0,0}; // magnet position in grid
grid_t graveyard_ptr = {0,0}; // grid i,j of next captured piece
grid_t grid[10][10] = { // y,x in mm to each square (from origin)
/*   x -->
/* y
/* |
/* V
 */
/*		0         1			2		   3		  4			 5			6		   7		  8			 9		  */
/* 0 */ {{0,0},   {0,68},    {0,133},   {0,196},   {0,259},   {0,322},   {0,386},   {0,450},   {0,513},   {0,577}},
/* 1 */ {{63,0},  {63,68},  {63,133},  {63,197},  {63,260},  {63,323},  {63,386},  {63,449},  {63,512},  {63,576}},
/* 2 */	{{128,0}, {126,64}, {128,128}, {128,192}, {128,256}, {128,320}, {128,384}, {128,448}, {128,512}, {128,576}},
/* 3 */	{{190,0}, {189,64}, {192,128}, {192,192}, {192,256}, {192,320}, {192,384}, {192,448}, {192,512}, {192,576}},
/* 4 */	{{254,0}, {252,64}, {256,128}, {256,192}, {256,256}, {256,320}, {256,384}, {256,448}, {256,512}, {256,576}},
/* 5 */	{{317,0}, {315,64}, {320,128}, {320,192}, {320,256}, {320,320}, {320,384}, {320,448}, {320,512}, {320,576}},
/* 6 */	{{383,0}, {378,64}, {384,128}, {384,192}, {384,256}, {384,320}, {384,384}, {384,448}, {384,512}, {384,576}},
/* 7 */	{{449,0}, {443,64}, {448,128}, {448,192}, {448,256}, {448,320}, {448,384}, {448,448}, {448,512}, {448,576}},
/* 8 */	{{512,0}, {506,64}, {512,128}, {512,192}, {512,256}, {512,320}, {512,384}, {512,448}, {512,512}, {512,576}},
/* 9 */	{{572,0}, {572,64}, {576,128}, {576,192}, {576,256}, {576,320}, {576,384}, {576,448}, {576,512}, {576,576}},
};

// returns where the next captured should go in slot.x and slot.y
// if slot.x and slot.y are -1, there are no more available slots
void get_current_graveyard_slot(grid_t *slot) {
	slot->x = graveyard_ptr.x;
	slot->y = graveyard_ptr.y;
	
	if (graveyard_ptr.x == -1 || graveyard_ptr.y == -1)
		return;
	
	// fills pieces up around the border, starting at (0,0) and 
	// moving up the y-axis, i.e. clockwise
	if (graveyard_ptr.x == 0 && graveyard_ptr.y < 9) {
		graveyard_ptr.y++;
	} else if (graveyard_ptr.y == 9 && graveyard_ptr.x < 9) {
		graveyard_ptr.x++;
	} else if (graveyard_ptr.x == 9 && graveyard_ptr.y > 0) {
		graveyard_ptr.y--;
	} else if (graveyard_ptr.y == 0 && graveyard_ptr.x > 0) {
		graveyard_ptr.x--;
	} 
	
	// no more slots available
	if (graveyard_ptr.x == 0 && graveyard_ptr.y == 0) {
		graveyard_ptr.x = -1;
		graveyard_ptr.y = -1;
	}
}

/* wrapper for inializing steps queue */
void step_control_init(void) {init(&steps);}
/* wrapper for accessing steps queue */
void add_to_queue(int16_t x, int16_t y) {add(&steps, x, y, NULL);}
/* wrapper for accessing steps queue */
void add_to_queue_d(int16_t x, int16_t y, done_func d) {add(&steps, x, y, d);}
/* wrapper for clearing steps queue */
void empty_queue(void) {clear_queue(&steps);}

int magnet_on(void) {
	LOG_TRACE("Turning magnet on");
	//MAGNET_ON;
	return 0;
} 

int magnet_off(void) {
	LOG_TRACE("Turning magnet off");
	int x = get_pos(X);
	int y = get_pos(Y);
	LOG_TRACE("(%d,%d)", x, y);
	//MAGNET_OFF; 
	return 0;
}

int move_done(void) {
	magnet_off();
	SEND_CMD_P(CMD_STATUS, "%d", OKAY);
	return 0;
}

int set_origin(void) {
	step_reset();
	pos.x = pos.y = 0;
	int x = get_pos(X);
	int y = get_pos(Y);
	LOG_TRACE("(%d,%d)", x, y);
	SEND_CMD_P(CMD_STATUS, "%d", OKAY);
	cal = 0;
	return 0;
}

void debug_move(int16_t x, int16_t y) {
	LOG_TRACE("debug_move");
	LOG_TRACE("get_pos(X)=%d, get_pos(Y)=%d", get_pos(X), get_pos(Y));
	LOG_TRACE("pos.x=%d, pos.y=%d", pos.x, pos.y);
	LOG_TRACE("grid[y][x].x=%d, grid[y][x].y=%d", grid[y][x].x, grid[y][x].y);
	int16_t dest_x = grid[y][x].x - get_pos(X);
	int16_t dest_y = grid[y][x].y - get_pos(Y);
	LOG_TRACE("dest_x=%d, dest_y=%d", dest_x, dest_y);
	add_to_queue_d(dest_x-(2*x), 0, magnet_off); // goto src
	add_to_queue_d(0, dest_y-(2*y), magnet_off); // goto src
	pos.x = x;
    pos.y = y; // update position now
}

/*
 *  move chess piece at (x,y) to (dest_x,dest_y)
 *	x, y, dest_x, dest_y are in mm
 */
void move_piece_mm(int16_t x, int16_t y, int16_t dest_x, int16_t dest_y) {
	 LOG_TRACE("move_piece: x=%d, y=%d, dest_x=%d, dest_y=%d", x, y, dest_x, dest_y);
	 // goto src
	 int16_t x_align = grid[y][x].x - pos.x;
	 int16_t y_align = grid[y][x].y - pos.y;
	 // move to dst
	 int16_t dx = grid[dest_y][dest_x].x - grid[y][x].x;
	 int16_t dy = grid[dest_y][dest_x].y - grid[y][x].y;
	
	 LOG_TRACE("x_align=%d, y_align=%d, dx=%d, dy=%d", x_align, y_align, dx, dy);
	 int16_t x_off, y_off;
     if (x < 9)
         x_off = (grid[y][x].x - grid[y][x+1].x)/2;
     else
         x_off = 32;
     if (y < 9)
         y_off = (grid[y][x].y - grid[y+1][x].y)/2;
     else
         y_off = 32;
	 
	 LOG_TRACE("x_off=%d, y_off=%d", x_off, y_off);
	 add_to_queue_d(x_align, y_align, magnet_on); // goto src
	 add_to_queue(x_off, y_off); // move piece onto line
	 add_to_queue(dx, 0); // move to dest, taxi-cab style
	 add_to_queue(0, dy);
	 add_to_queue_d(-1*x_off, -1*y_off, move_done); // stagger off line
     pos.x = dest_x;
     pos.y = dest_y; // update position now
}

/*
 *  move chess piece at x,y to dest_x, dest_y
 *	x, y, dest_x, dest_y are in squares
 */
void move_piece(uint8_t x, uint8_t y, uint8_t dest_x, uint8_t dest_y) {
	 move_piece_mm(SQUARES_TO_MM(x), SQUARES_TO_MM(y), SQUARES_TO_MM(dest_x), SQUARES_TO_MM(dest_y));
}

#define COORD_INVALID(c) ((c) > 7)
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
	char src_x, src_y, dst_x, dst_y;
	
	if(strlen(move) == 4) {
		SET_COORDS(src_x, src_y, move);
		SET_COORDS(dst_x, dst_y, move+2);
	} else {
		return -1;
	}

	if (COORD_INVALID(src_x) || COORD_INVALID(src_y) || COORD_INVALID(dst_x) || COORD_INVALID(dst_y))
		return -1;
	
	move_piece_mm(src_x, src_y, dst_x, dst_y);
	return 0;
}
#undef COORD_INVALID

/*
 *  update event interrupt, controls continous stepping
 */
void TIM2_IRQHandler(void) {
	// if not stepping, get next step from queue
	if (!stepping()) {
		if (current.done) { // run done function
			current.done();
			current.done = NULL;
		}
		
		if (!is_empty(&steps)) {
			current = rm(&steps);
		
			step_mm(X, current.x);
			step_mm(Y, current.y);
		}
	} else {
		step();
	}
	TIM2->SR &= ~(TIM_SR_UIF); // clear interrupt
}

unsigned char calibrating(void) {
	return cal;
}

int calibrate(void) {
	// step until hit calibration switches
	cal = 1;
	unsigned long timeout = systime + CALIBRATION_TIMEOUT;
	LOG_TRACE("systime=%ld", systime);
	LOG_TRACE("timeout=%ld", timeout);
	unsigned char x_done = 0;
	unsigned char y_done = 0;
	int ret = 0;
	add_to_queue(-2000, -2000);
	add_to_queue_d(ORIGIN_X, ORIGIN_Y, set_origin);
	
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
		} else if (systime > timeout) { // timeout
			stop_stepping();
			empty_queue();
			ret = -1;
			break;
		}
    }
	LOG_TRACE("done calibrating");
	return ret;
}

/*
 *	for user button (kill switch)
 */
void HAL_SYSTICK_Callback(void) {
	systime++;
    static uint32_t debouncer = 0;
	static uint8_t x_debouncer = 0;
	static uint8_t y_debouncer = 0;
    
    debouncer = (debouncer << 1);
    if(GPIOA->IDR & (1 << 0)) {
        debouncer |= 0x1;
    }

    if(debouncer == 0x7FFFFFFF) {
		if (stepping()) { // kill switch
			stop_stepping();
			empty_queue();
		} else { // send ok to photon
			SEND_CMD_P(CMD_STATUS, "%d", OKAY);
		}
    }
	
	if (!calibrating()) {
		if (axis_stepping(X)) {
			x_debouncer = (x_debouncer << 1);
			if (GPIOC->IDR & (1 << X_CAL)) {
				x_debouncer |= 0x1;
			}
			if (x_debouncer == 0x7F) {
				x_debouncer = 0;
				stop_stepping();
				empty_queue();
				//add_to_queue(SQUARE_HALF_WIDTH, 0);
			}
			if (get_pos(X) < LOWER_LIMIT || get_pos(X) > UPPER_LIMIT) {
				stop_stepping();
				empty_queue();
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
				empty_queue();
				//add_to_queue(0, SQUARE_HALF_WIDTH);
			}
			if (get_pos(Y) < LOWER_LIMIT || get_pos(Y) > UPPER_LIMIT) {
				stop_stepping();
				empty_queue();
			}
		}
	}
    
}
