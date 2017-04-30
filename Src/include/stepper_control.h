#ifndef __STEPPER_CTRL_H_
#define __STEPPER_CTRL_H_

// chess specific
#define SQUARE_WIDTH 51 // mm
#define SQUARE_HALF_WIDTH 26 // mm
#define ASCII_OFFSET_a 97 // 'a'
#define ASCII_OFFSET_0 49 // '0'-1 (for proper index, convert 1->0)

void step_squares(int axis, int n);
void move_piece(int x, int y, int dest_x, int dest_y);

#endif /* __STEPPER_CTRL_H_ */