#ifndef HALL_ARRAY_LIBRARY
#define HALL_ARRAY_LIBRARY

#include <stdint.h>
#include <stdlib.h>
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "main.h"

#define NUM_SQUARES 64
#define ROW_COL 8
//#define THRESHOLD 115


typedef struct{
	int16_t buffer[8][8];	
}board_buffer;

void initialize_biases(void);

board_buffer get_board_state(void);
void transmit_char_usart(char);
void transmit_voltage_usart(uint16_t);
void transmit_string_usart(char*);
void pseudo_main(void);
board_buffer scan_bools(board_buffer*, board_buffer*);

//float adcval_tovolt(uint16_t);
board_buffer scan_array(board_buffer*);
board_buffer check_three_boards(void);
char* itoa(int num, char* str, int base);
void reverse(char str[], int length);
void swap(char* a, char* b);
void print_bools(void);
void print_state(void);
void print_biases(void);
int16_t take_reading(void);

void turn_on_a(void);
void turn_on_b(void);
void turn_on_c(void);
void turn_on_d(void);
void turn_on_e(void);
void turn_on_f(void);

void turn_off_a(void);
void turn_off_b(void);
void turn_off_c(void);
void turn_off_d(void);
void turn_off_e(void);
void turn_off_f(void);

void turn_off_all(void);

#endif
