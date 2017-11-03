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

/* wrapper for inializing steps queue */
void step_control_init(void) {init(&steps);}
/* wrapper for accessing steps queue */
void add_to_queue(int x, int y) {add(&steps, x, y, NULL);}
/* wrapper for accessing steps queue */
void add_to_queue_d(int x, int y, done_func d) {add(&steps, x, y, d);}
/* wrapper for clearing steps queue */
void empty_queue(void) {clear_queue(&steps);}

int magnet_on(void) {
	LOG_TRACE("Turning magnet on");
	MAGNET_ON;
	return 0;
}

int magnet_off(void) {
	LOG_TRACE("Turning magnet off");
	int x = get_pos(X);
	int y = get_pos(Y);
	LOG_TRACE("(%d,%d)", x, y);
	MAGNET_OFF; 
	return 0;
}

int move_done(void) {
	magnet_off();
	//SEND_CMD_P(CMD_STATUS, "%d", OKAY);
	return 0;
}

int set_origin(void) {
	step_reset();
	int x = get_pos(X);
	int y = get_pos(Y);
	LOG_TRACE("(%d,%d)", x, y);
	//SEND_CMD_P(CMD_STATUS, "%d", OKAY);
	cal = 0;
	return 0;
}

/*
 *  move chess piece at x,y to dest_x, dest_y
 *	x, y, dest_x, dest_y are in squares
 */
void move_piece(int x, int y, int dest_x, int dest_y) {
	 // goto src
	 int x_align = SQUARES_TO_MM(x) - get_pos(X); // in mm
	 int y_align = SQUARES_TO_MM(y) - get_pos(Y);
	 
	 // move to dst
	 int x_mm = SQUARES_TO_MM(dest_x - x);
	 int y_mm = SQUARES_TO_MM(dest_y - y);
	
	 add_to_queue_d(x_align, y_align, magnet_on); // goto src
	 add_to_queue(HALF_SQUARES_TO_MM(1), HALF_SQUARES_TO_MM(1)); // move piece onto line
	 add_to_queue(x_mm, 0); // move to dest, taxi-cab style
	 add_to_queue(0, y_mm);
	 add_to_queue_d(HALF_SQUARES_TO_MM(-1), HALF_SQUARES_TO_MM(-1), move_done); // stagger off line
}

#define SET_COORDS(sx, sy, dx, dy) \
do { \
	src_x = move[(sx)]-'a'; \
	src_y = move[(sy)]-'1'; \
	dst_x = move[(dx)]-'a'; \
	dst_y = move[(dy)]-'1'; \
} while(0)

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
	char type;
	char capture = 0;
	char promote = 0;
	
	switch(strlen(move)) {
		case 4: // ssdd
			type = 'P';
			SET_COORDS(0, 1, 2, 3);
			break;
		case 5: // Tssdd or ssxdd or ssddP
			if (move[0] == 'K' || move[0] == 'Q' || move[0] == 'R' || // Tssdd
				move[0] == 'B' || move[0] == 'N') {
				type = move[0];
				SET_COORDS(1, 2, 3, 4);
			} else if (move[2] == 'x') { // ssxdd
				capture = 1;
				type = 'P';
				SET_COORDS(0, 1, 3, 4);
			} else { // ssddP
				promote = move[4];
				type = 'P';
				SET_COORDS(0, 1, 2, 3);
			}
			break;
		case 6: // Tssxdd
			capture = 1;
			type = move[0];
			SET_COORDS(1, 2, 4, 5);
			break;
		default: // invalid
			return -1;
	}

	if (COORD_INVALID(src_x) || COORD_INVALID(src_y) || COORD_INVALID(dst_x) || COORD_INVALID(dst_y))
		return -1;
	
	move_piece(src_x, src_y, dst_x, dst_y);
	return 0;
}

#undef SET_COORDS
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
	unsigned long timeout = systime + 10000;
	LOG_TRACE("systime=%ld", systime);
	LOG_TRACE("timeout=%ld", timeout);
	unsigned char x_done = 0;
	unsigned char y_done = 0;
	int ret = 0;
	int origin = SQUARE_WIDTH + SQUARE_HALF_WIDTH;
	add_to_queue(-2000, -2000);
	add_to_queue_d(origin, origin, set_origin);
	
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
			if (/*get_pos(X) < LOWER_LIMIT ||*/ get_pos(X) > UPPER_LIMIT) {
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
			if (/*get_pos(Y) < LOWER_LIMIT ||*/ get_pos(Y) > UPPER_LIMIT) {
				stop_stepping();
				empty_queue();
			}
		}
	}
    
}
