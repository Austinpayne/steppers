#include "include/stepper.h"
#include "include/stepper_control.h"
#include "include/queue.h"

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
 *	wrapper for accessing steps queue
 */
void empty_queue(void) {
	clear_queue(&steps);
}

/*
 *	squares are 2" or ~51mm
 */
void step_squares(int axis, int n) {
	step_mm(axis, squares_to_mm(n));
}

/*
 *  move chess piece at x,y to dest_x, dest_y
 *	x, y, dest_x, dest_y are in squares
 */
void move_piece(int x, int y, int dest_x, int dest_y) {
	 // goto src
	 int x_align = squares_to_mm(x) - get_pos(X); // in mm
	 int y_align = squares_to_mm(y) - get_pos(Y);
	
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
	 
	 int x_mm = squares_to_mm(x_squares);
	 int y_mm = squares_to_mm(y_squares);
	 
	 add(&steps, x_align, y_align); // goto src
	 add(&steps, x_offset*SQUARE_HALF_WIDTH, y_offset*SQUARE_HALF_WIDTH); // stagger onto line
	 add(&steps,  x_mm, 0); // move to dest, taxi-cab style
	 add(&steps, 0, y_mm);
	 add(&steps, x_offset*SQUARE_HALF_WIDTH, y_offset*SQUARE_HALF_WIDTH); // stagger off line
	 
	 // turn off electromagnet (after move)
}

/*
 *  Universal Chess Interface (UCI) move
 *  move is in the form [src_col][src_row][dst_col][dst_row},
 *  example:	e2e4 (pawn do not indicate piece)
 *				Nd3d7 (Night at d3 to d7)
 */
void uci_move(const char *move) {
	// TODO: add a lot more error checking
	int src_x = move[0]-ASCII_OFFSET_a;
	int src_y = move[1]-ASCII_OFFSET_0;
	int dst_x = move[2]-ASCII_OFFSET_a;
	int dst_y = move[3]-ASCII_OFFSET_0;
	move_piece(src_x, src_y, dst_x, dst_y);
}

/*
 *	squares = mm/SQUARE_WIDTH
 */
int mm_to_squares(int mm) {
	int squares = mm/SQUARE_WIDTH;
	return squares;
}

/*
 *	mm = SQUARE_WIDTH*squares
 */
int squares_to_mm(int squares) {
	int mm = SQUARE_WIDTH*squares;
	return mm;
}

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
