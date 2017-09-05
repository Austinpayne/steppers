#ifndef __STEPPER_CTRL_H_
#define __STEPPER_CTRL_H_

// chess board constants
#define SQUARE_WIDTH 51 // mm
#define SQUARE_HALF_WIDTH 26 // mm

#define MAGNET_OFF_OFF 0x0 // 00
#define MAGNET_OFF_ON  0x1 // 01
#define MAGNET_ON_OFF  0x2 // 10
#define MAGNET_ON_ON   0x3 // 11

#define MAGNET_START_ON 0x2 // 01
#define MAGNET_END_ON   0x1 // 01

// macros
#define SQUARES_TO_MM(squares)     (SQUARE_WIDTH*squares)
#define HALF_SQUARES_TO_MM(halves) (SQUARE_HALF_WIDTH*halves)

// func definitions
void step_control_init(void);
void step_squares(int axis, int n);
void move_piece(int x, int y, int dest_x, int dest_y);
int  uci_move(const char *move);
void add_to_queue(int x, int y, char magnet_on);
void empty_queue(void);

#endif /* __STEPPER_CTRL_H_ */
