#ifndef __STEPPER_CTRL_H_
#define __STEPPER_CTRL_H_

// chess board constants
#define SQUARE_WIDTH 51 // mm
#define SQUARE_HALF_WIDTH 26 // mm
#define ASCII_OFFSET_a 97 // 'a'
#define ASCII_OFFSET_0 49 // '1' (for proper index, convert 1->0)

// func definitions
void step_control_init(void);
void step_squares(int axis, int n);
void move_piece(int x, int y, int dest_x, int dest_y);
void uci_move(const char *move);
int  mm_to_squares(int mm);
int  squares_to_mm(int squares);
void add_to_queue(int x, int y);
void empty_queue(void);

#endif /* __STEPPER_CTRL_H_ */
