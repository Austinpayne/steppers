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