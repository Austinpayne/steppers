#ifndef HALL_ARRAY_LIBRARY
#define HALL_ARRAY_LIBRARY

#include <stdint.h>
#include <stdlib.h>
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "main.h"

#define NUM_SQUARES 64
#define ROW_COL 8
#define THRESHOLD 420
#define pieces_on_board 32

#define turn_on_a()  (GPIOB->BSRR |= GPIO_BSRR_BS_10)
#define turn_on_b()  (GPIOB->BSRR |= GPIO_BSRR_BS_11)
#define turn_on_c()  (GPIOB->BSRR |= GPIO_BSRR_BS_12)
#define turn_on_d()  (GPIOB->BSRR |= GPIO_BSRR_BS_13)
#define turn_on_e()  (GPIOB->BSRR |= GPIO_BSRR_BS_14)
#define turn_on_f()  (GPIOB->BSRR |= GPIO_BSRR_BS_15)
#define turn_off_a() (GPIOB->BSRR |= GPIO_BSRR_BR_10)
#define turn_off_b() (GPIOB->BSRR |= GPIO_BSRR_BR_11)
#define turn_off_c() (GPIOB->BSRR |= GPIO_BSRR_BR_12)
#define turn_off_d() (GPIOB->BSRR |= GPIO_BSRR_BR_13)
#define turn_off_e() (GPIOB->BSRR |= GPIO_BSRR_BR_14)
#define turn_off_f() (GPIOB->BSRR |= GPIO_BSRR_BR_15)

typedef struct{
	int16_t buffer[8][8];	
}board_buffer;

typedef struct{
	char buf[5];
}move_string;

void initialize_biases(void);

void get_board_state(void);
//void transmit_voltage_usart(uint16_t );
void pseudo_main(void);

int8_t calculate_move(board_buffer*, board_buffer*, move_string*);
void scan_bools(board_buffer*);


void zero_out_board(board_buffer*);
//float adcval_tovolt(uint16_t );
void scan_array(volatile board_buffer*);
void check_three_boards(board_buffer* board_1, board_buffer* board_2);
void print_board(board_buffer*);
int16_t count_pieces(board_buffer*);
//void rebias_chessboard();
//void print16_t _bools(void);
//void print16_t _state(void);
//void print16_t _biases(void);
int16_t take_reading(void);

void turn_off_all(void);
void init_board(void);

#endif
