#include "include/stepper.h"
#include "include/stepper_control.h"
#include "include/queue.h"

step_queue_t steps;

void step_control_init(void) {
	init(&steps);
}

// squares are 2" or ~51mm
void step_squares(int axis, int n) {
	step_mm(axis, SQUARE_WIDTH*n);
}

void move_piece(int x, int y, int dest_x, int dest_y) {
	 // goto src
	 int x_align = get_pos(X) - x;
	 int y_align = get_pos(Y) - y;
	 add(&steps, x_align, y_align);
	
	 // turn on electromagnet (after move)
	
	 // move to dst
	 int x_offset = 0;
	 int y_offset = 0;
	 int x_squares = dest_x - x;
	 int y_squares = dest_y - y;
	 
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
	 
	 //add(&steps, x_offset*SQUARE_HALF_WIDTH, y_offset*SQUARE_HALF_WIDTH); // stagger onto line
	 add(&steps, x_squares , 0);
	 add(&steps, 0, y_squares);
	 //add(&steps, x_offset*SQUARE_HALF_WIDTH, y_offset*SQUARE_HALF_WIDTH); // stagger off line
	 
	 // turn off electromagnet (after move)
}

// e2e4
void uci_move(const char *move) {
	int src_x = move[0]-ASCII_OFFSET_a;
	int src_y = move[1]-ASCII_OFFSET_0;
	int dst_x = move[2]-ASCII_OFFSET_a;
	int dst_y = move[3]-ASCII_OFFSET_0;
	move_piece(src_x, src_y, dst_x, dst_y);
}

void TIM2_IRQHandler(void) {
	// if not stepping, get next step from queue
	if (!empty(&steps) && get_steps(X) == OFF && get_steps(Y) == OFF) {
		steps_t next = rm(&steps);
		step_squares(X, next.x_steps);
		step_squares(Y, next.y_steps);
	} else {
		step();
	}
}