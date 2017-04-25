#include "include/stepper.h"
#include "include/stepper_control.h"

// squares are 2" or ~51mm
void step_squares(int axis, int n) {
	step_mm(axis, SQUARE_WIDTH*n);
}

void move_piece(int x, int y, int dest_x, int dest_y) {
	 
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
	 //add(&steps, x_squares , 0);
	 //add(&steps, 0, y_squares);
	 //add(&steps, x_offset*SQUARE_HALF_WIDTH, y_offset*SQUARE_HALF_WIDTH); // stagger off line
}

// e2e4
void uci_move_to_coords(const char *move) {
	int src_x = move[0]-FILE_ASCII_OFFSET;
	int src_y = move[1]-1;
	int dst_x = move[2]-FILE_ASCII_OFFSET;
	int dst_y = move[3]-1;
	//move_piece(src_x, src_y, dst_x, dst_y);
}