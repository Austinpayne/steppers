#ifndef __STEPPER_CTRL_H_
#define __STEPPER_CTRL_H_

// chess board constants
#define SQUARE_WIDTH 62 // mm
#define SQUARE_HALF_WIDTH 31 // mm
#define X_WIDTH_ERR 0 // mm
#define Y_WIDTH_ERR 2 // mm
#define LOWER_LIMIT 0-(SQUARE_WIDTH+SQUARE_HALF_WIDTH)// mm
#define UPPER_LIMIT 600 // mm
#define ORIGIN_X 41 // mm
#define ORIGIN_Y 25 // mm
#define CALIBRATION_TIMEOUT 10000 // ms

// macros
#define SQUARES_TO_MM(squares)     (SQUARE_WIDTH*(squares))
#define HALF_SQUARES_TO_MM(halves) (SQUARE_HALF_WIDTH*(halves))

// magnet
#define MAGNET_PIN 1 // GPIOC
#define MAGNET_OFF (GPIOC->ODR &= ~(1 << MAGNET_PIN))
#define MAGNET_ON (GPIOC->ODR  |=  (1 << MAGNET_PIN))

// movement
#define SET_COORDS(x,y,uci) do {(x)=(uci)[0]-'a'+1; (y)=(uci)[1]-'1'+1;} while(0)
#define COORD_INVALID(c) ((c) < 1 || (c) > 8)
#define MOVE_PIECE_TO_GRAVEYARD(x,y,g,c) do { \
	if ((c) == 'w' || (c) == 'b') { \
		get_current_graveyard_slot(&(g), (c)); \
		move_piece_to_mm((x), (y), (g).x, (g).y); \
	} \
} while(0)
#define TAXI_CAB_MOVE(sx,sy,ox,oy,dx,dy) do { \
	step_mm_blocking((sx), 0); \
	step_mm_blocking(0, (sy)); \
	magnet_on(); \
	step_mm_blocking((ox), 0); \
	step_mm_blocking(0, (oy)); \
	step_mm_blocking((dx), 0); \
	step_mm_blocking(0, (dy)); \
	step_mm_blocking(-1*(ox), 0); \
	step_mm_blocking(0, -1*(oy)); \
	move_done(); \
} while(0)
#define DIFF_MM(dst,src) ((dst) - (src))
#define SET_OFFSET(ox,oy) do { \
	if (x < N-1) \
		ox = (grid[y][x].x - grid[y][x+1].x)/2; \
	else \
		ox = 32; \
	if (y < N-1) \
		oy = (grid[y][x].y - grid[y+1][x].y)/2; \
	else \
		oy = 32; \
} while(0)

typedef struct {
	int16_t y;
	int16_t x;
} grid_t;

// func definitions
int get_current_graveyard_slot(grid_t *slot, char color);
void step_control_init(void);
void step_squares(int axis, int n);
void debug_squares(void);
void debug_move(int16_t x, int16_t y);
void move_piece_to_mm(uint8_t x, uint8_t y, int16_t dest_x, int16_t dest_y);
void move_piece(int16_t x, int16_t y, int16_t dest_x, int16_t dest_y);
int  uci_move(const char *move);
int magnet_on(void);
int magnet_off(void);
int move_done(void);
int set_origin(void);
unsigned char calibrating(void);
int calibrate(void);

#endif /* __STEPPER_CTRL_H_ */
