#ifndef __STEPPER_CTRL_H_
#define __STEPPER_CTRL_H_

// chess board constants
#define SQUARE_WIDTH 51 // mm
#define SQUARE_HALF_WIDTH 26 // mm

// macros
#define SQUARES_TO_MM(squares)     (SQUARE_WIDTH*squares)
#define HALF_SQUARES_TO_MM(halves) (SQUARE_HALF_WIDTH*halves)

// func definitions
void step_control_init(void);
void step_squares(int axis, int n);
void move_piece(int x, int y, int dest_x, int dest_y);
int  uci_move(const char *move);
void add_to_queue(int x, int y);
void empty_queue(void);

#endif /* __STEPPER_CTRL_H_ */
