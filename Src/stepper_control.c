#include "include/stepper.h"
#include "include/stepper_control.h"
#include "include/queue.h"
#include "string.h"

tuple_queue_t steps; // in mm (makes stepping half squares more accurate)

/*
 *	inialize steps queue
 */
void step_control_init(void) {
	init(&steps);
}

/*
 *	wrapper for accessing steps queue
 */
void add_to_queue(int x, int y) {
	add(&steps, x, y);
}

/*
 *	wrapper for clearing steps queue
 */
void empty_queue(void) {
	clear_queue(&steps);
}

/*
 *  move chess piece at x,y to dest_x, dest_y
 *	x, y, dest_x, dest_y are in squares
 */
void move_piece(int x, int y, int dest_x, int dest_y) {
	 // goto src
	 int x_align = SQUARES_TO_MM(x) - get_pos(X); // in mm
	 int y_align = SQUARES_TO_MM(y) - get_pos(Y);
	
	 // turn on electromagnet (after move)
	
	 // move to dst
	 int x_offset = 0;
	 int y_offset = 0;
	 int x_squares = dest_x - x;
	 int y_squares = dest_y - y;
	 
	// for offsetting onto/off line
	 if (x_squares > 0) { // +x direction
		 x_squares--;
		 x_offset = 1;
	 } else if (x_squares < 0) { // -x direction
		 x_squares++;
		 x_offset = -1;
	 }
	 
	 if (y_squares > 0) { // +y direction
		 y_squares--;
		 y_offset = 1;
	 } else if (y_squares < 0) { // -y direction
		 y_squares++;
		 y_offset = -1;
	 }
	 
	 int x_mm = SQUARES_TO_MM(x_squares);
	 int y_mm = SQUARES_TO_MM(y_squares);
	 
	 add_to_queue(x_align, y_align); // goto src
	 add_to_queue(HALF_SQUARES_TO_MM(x_offset), HALF_SQUARES_TO_MM(y_offset)); // stagger onto line
	 add_to_queue(x_mm, 0); // move to dest, taxi-cab style
	 add_to_queue(0, y_mm);
	 add_to_queue(HALF_SQUARES_TO_MM(x_offset), HALF_SQUARES_TO_MM(y_offset)); // stagger off line
	 
	 // turn off electromagnet (after move)
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
	if (!is_empty(&steps) && get_steps(X) == OFF && get_steps(Y) == OFF) {
		tuple_t next = rm(&steps);
		step_mm(X, next.x);
		step_mm(Y, next.y);
	} else {
		step();
	}
}
