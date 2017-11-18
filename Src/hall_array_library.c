#include "hall_array_library.h"

//board_buffer current_biases;
//board_buffer cur_state;
//board_buffer magnet_pos;

const int16_t THRESHOLD = 100;
int16_t pieces_on_board = 32;
//const int16_t THRESHOLD = 100;

void swap(char* a, char* b)
{
 int16_t temp = *a;
 *a = *b;
 *b = temp;
}

int16_t count_pieces(board_buffer* board){
	int16_t result = 0;
	for(int i = 0; i < 8; i++){
		for(int j = 0; j < 8; j++){
			if(board->buffer[i][j] == 1)
				result ++;
		}
	}
	return result;
}

/* A utility function to reverse a string  */
void reverse(char str[], int16_t length){
    int16_t start = 0;
    int16_t end = length -1;
    while (start < end){
        swap((str+start), (str+end));
        start++;
        end--;
    }
}
void scan_bools(board_buffer* state, board_buffer* biases, board_buffer* bools){
	unsigned row = 0, col = 0;
	//board_buffer bools;
	for(int16_t i = 0; i < 8; i++){
		for(int16_t j = 0; j < 8; j++){
			int16_t current_bias_val = biases->buffer[i][j];
			int16_t cur_state_val = state->buffer[i][j];
			//int16_t current_bias_val = current_biases.buffer[i][j];
			//int16_t cur_state_val = cur_state.buffer[i][j];
			int16_t current_diff = cur_state_val - current_bias_val;
			if(current_diff > THRESHOLD){
				bools->buffer[i][j] = 1;
			}else{
				bools->buffer[i][j] = 0;
			}
		}
	}
		return;
}
// Implementation of itoa()
char* itoa(int16_t num, char* str, int16_t base){
    int16_t i = 0;
    char isNegative = 0;
    /* Handle 0 explicitely, otherwise empty string is printed for 0 */
    if (num == 0){
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }
    // In standard itoa(), negative numbers are handled only with 
    // base 10. Otherwise numbers are considered unsigned.
    if (num < 0 && base == 10){
        isNegative = 1;
        num = -num;
    }
    // Process individual digits
    while (num != 0){
        int16_t rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
        num = num/base;
    }
    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';
    str[i] = '\0'; // Append string terminator
    // Reverse the string
    reverse(str, i);
    return str;
}

board_buffer get_board_state(){
	board_buffer chess_board;
	return chess_board;
}

void transmit_string_usart(char* str){
	char c = str[0];
	unsigned i = 0;
	while(c != '\0'){
		transmit_char_usart(c);
		i ++;
		c = str[i];
	}
	return;
}

void transmit_char_usart(char c){
	while((USART1->ISR & USART_ISR_TXE) == 0){
		//enable timeout
	}
	USART1->TDR = c;
}
float adcval_tovolt(int16_t adc_val){
	float val = 0;
	float temp = 3.3/4096;
	val =((float)adc_val)*temp;
	return temp;
}

/*void transmit_voltage_usart(unsigned int16_t val){
	
	unsigned int16_t mask = 0xff00;
	unsigned int16_t tx = 0;
	unsigned int8_t transmit = 0;
	for(unsigned i = 0; i < 2; i ++){
		tx = mask & val;
		mask = mask >> 8;
		tx  = tx >> (8 * (1 - i));
		transmit = (unsigned int8_t) tx;
		transmit_char_usart((char)(transmit + '0'));
	}
	transmit_char_usart('\r');
	transmit_char_usart('\n');
}*/

void scan_array(board_buffer* buf){
	//unsigned int16_t array_values[64][64];
	//board_buffer state;
	int16_t row = 0, col = 0;
	for(unsigned i = 0; i < 4; i++){
		switch (i){
			case 0:
				turn_off_all();
				col = 0;
			break;
			case 1:
				turn_off_all();
				turn_on_a();
				col = 2;
			break;
			case 2:
				turn_off_all();
				turn_on_b();
				col = 4;
			break;
			case 3:
				turn_off_all();
				turn_on_a();
				turn_on_b();
				col = 6;
			break;
			default:
			break;
		}
		//HAL_Delay(5);
		for(unsigned j = 0; j < 16; j++){
			row = j % 8;
			if(j == 8){
				col ++;
			}
			if(j < 8){
				turn_off_f();
			}else{
				turn_on_f();
			}
			unsigned mod = j % 8;
			if(mod < 4){
				turn_off_e();
			}else{
				turn_on_e();
			}
			mod = j % 4;
			if(mod < 2){
				turn_off_d();
			}else{
				turn_on_d();
			}
			mod = j % 2;
			if(mod == 0){
				turn_off_c();
			}else{
				turn_on_c();
			}
		//	for(int32_t k = 0; k < 10000; k++){
		//		__nop();
		//	}
			HAL_Delay(5);
			int16_t reading = take_reading();
			buf->buffer[row][col] = reading;
			
			int16_t string_int16_t = reading & 0x0fff;
		}
	}
	return;
}
int16_t take_reading(void){
	int16_t result = 0;
	int16_t reading = 0;
	for(unsigned j = 0; j < 2; j++){
		while((ADC1->ISR & ADC_ISR_EOC) == 0){
		//implement time-out
		}
		reading = ADC1->DR & 0xfff;
		reading = 0;
	}
	for(unsigned i = 0; i < 5; i++){
		while((ADC1->ISR & ADC_ISR_EOC) == 0){
		//implement time-out
		}
		reading = ADC1->DR & 0xfff;
		result += reading;
	}
	result = result / 5;
	return result;
}

void print_board(board_buffer board){
	for(int16_t i = 7; i >= 0; i--){// i is the row
		for(int16_t j = 0; j < 8; j++){// j is the col
			int16_t string_int = board.buffer[i][j];
			char buffer [sizeof(unsigned int)*8+1];
			itoa(string_int,buffer,10);
			transmit_string_usart(buffer);
			transmit_char_usart(' ');
			if(j == 7){
				transmit_char_usart('\r');
				transmit_char_usart('\n');
			}
		}
	}
}

void pseudo_main(void){
	
	board_buffer* cur_state = malloc(sizeof(board_buffer));
	board_buffer* current_biases = malloc(sizeof(board_buffer));
	board_buffer* mag_pos = malloc(sizeof(mag_pos));
	//unsigned int16_t biases[64][64];
//	for(unsigned i = 0; i < NUM_SQUARES; i++){
//		for(unsigned j = 0; j < NUM_SQUARES; j++){
//			biases[i][j] = 0;
//		}
//	}
	//initialize_biases();
	char str[7];
	str[0] = 'c';
	str[1] = 'h';
	str[2] = 'e';
	str[3] = 's';
	str[4] = 's';
	str[5] = '\r';
	str[6] = '\n';
	//str[7] = '\0';
	for(unsigned i = 0; i < 7; i++){
		transmit_char_usart(str[i]);
	}
	
	//int16_t reading = 0;
	//look at mux 0,0,0,0,0,0 - A1
	GPIOB->BSRR |= GPIO_BSRR_BR_10;
	GPIOB->BSRR |= GPIO_BSRR_BR_11;
	GPIOB->BSRR |= GPIO_BSRR_BR_12;
	GPIOB->BSRR |= GPIO_BSRR_BR_13;
	GPIOB->BSRR |= GPIO_BSRR_BR_14;
	GPIOB->BSRR |= GPIO_BSRR_BR_15;
	
	//GPIOB->BSRR |= GPIO_BSRR_BS_12;

	ADC1->CR |= ADC_CR_ADSTART;//turn on adc
//	board_buffer temp;
//	current_biases = scan_array(temp);
	scan_array(current_biases);
	scan_array(cur_state);
	scan_array(current_biases);

	board_buffer board_state1, board_state2;

  while (1)
  {
		while((ADC1->ISR & ADC_ISR_EOC) == 0){
			//implement time-out
		}
		board_buffer board_1, board_2, board_3;
		zero_out_board(&board_1);
		zero_out_board(&board_2);
		zero_out_board(&board_3);
		//zero_out_board(&temp);
		//current_biases = scan_array(temp);
		//scan_array(&cur_state);
		//magnet_pos = scan_bools();
		board_state1 = check_three_boards(&board_1, &board_2, &board_3, cur_state,current_biases);
		print_board(board_state1);
		transmit_char_usart('\r');
		transmit_char_usart('\n');
		
		transmit_char_usart('5');
		transmit_char_usart('\r');
		transmit_char_usart('\n');
		HAL_Delay(1000);
		transmit_char_usart('4');
		transmit_char_usart('\r');
		transmit_char_usart('\n');
		HAL_Delay(1000);
		
		board_buffer board_state3 = check_three_boards(&board_1, &board_2, &board_3, cur_state, current_biases);
		transmit_char_usart('3');
		transmit_char_usart('\r');
		transmit_char_usart('\n');
		HAL_Delay(1000);		
		transmit_char_usart('2');
		transmit_char_usart('\r');
		transmit_char_usart('\n');
		HAL_Delay(1000);
		transmit_char_usart('1');
		transmit_char_usart('\r');
		transmit_char_usart('\n');
		transmit_char_usart('\r');
		transmit_char_usart('\n');
		HAL_Delay(1000);
		zero_out_board(&board_1); zero_out_board(&board_2); zero_out_board(&board_3);
		//scan_array(&cur_state);
		board_state2 = check_three_boards(&board_1, &board_2, &board_3, cur_state,current_biases);
		print_board(board_state2);
		transmit_char_usart('\r');
		transmit_char_usart('\n');
		
		move_string move;
		calculate_move(&board_state1, &board_state2, &move);
		transmit_string_usart(move.buf);
		transmit_char_usart('\r');
		transmit_char_usart('\n');
		//print_board(magnet_pos);
		/*PLACE PIECES NOW DISPLAY ON DISPLAY*/
		/*REPORT TO SERVER*/	
		//transmit_char_usart('\r');
		//transmit_char_usart('\n');
		//board_buffer buf1;
		//board_buffer buf2;
		//zero_out_board(&buf1);
		//zero_out_board(&buf2);
		//buf1.buffer[0][1] = 1;
		//buf2.buffer[2][2] = 1;
		//move_string move;
		//calculate_move(&buf1, &buf2, &move);
		HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}
void zero_out_board(board_buffer* board){
	for(unsigned i = 0; i < 8; i++){
		for(unsigned j = 0; j < 8; j++){
			board->buffer[i][j] = 0;
		}
	}
}
void calculate_move(board_buffer* prev_state, board_buffer* new_state, move_string* move_buf){
	unsigned p1x = 0, p1y = 0;
	unsigned p2x = 0, p2y = 0;
	unsigned found_first_point = 0;
	unsigned prev_x = 0, prev_y = 0;
	unsigned new_x = 0, new_y = 0;
	for(unsigned i = 0; i < 8; i++){
		for(unsigned j = 0; j < 8; j++){
			int16_t test = prev_state->buffer[i][j] ^ new_state->buffer[i][j];
			if(test){
				if(!found_first_point){
					p1x = j; p1y = i;
					found_first_point = 1;
				}else{
					p2x = j; p2y = i;
				}
			}
		}
	}
	if(new_state->buffer[p1y][p1x]){
		prev_x = p2x;
		prev_y = p2y;
		new_x = p1x;
		new_y = p1y;
	}else{
		prev_x = p1x;
		prev_y = p1y;
		new_x = p2x;
		new_y = p2y;
	}
	//char move[4];
	move_buf->buf[0] = prev_x + 'a';
	move_buf->buf[1] = prev_y + '1';
	move_buf->buf[2] = new_x + 'a';
	move_buf->buf[3] = new_y + '1';
	move_buf->buf[4] = '\0';
	return;
}
board_buffer check_three_boards(board_buffer* board_1, board_buffer* board_2,
																board_buffer* board_3, board_buffer* cur_state, board_buffer* current_biases){
	board_buffer temp;
	//temp = scan_array(&cur_state);
	scan_array(cur_state);
	scan_bools(cur_state, current_biases, board_1);
	scan_array(cur_state);
	scan_bools(cur_state, current_biases, board_2);
	scan_array(cur_state);
	scan_bools(cur_state, current_biases, board_3);
	
	for(unsigned i = 0; i < 8; i++){
		for(unsigned j = 0; j < 8; j++){
			if(board_1->buffer[i][j] && board_2->buffer[i][j] && board_3->buffer[i][j]){
				temp.buffer[i][j] = 1;
			}else{
				temp.buffer[i][j] = 0;
			}
		}
	}
	return temp;
}
void turn_on_a(void){
	GPIOB->BSRR |= GPIO_BSRR_BS_10;
}
void turn_on_b(void){
	GPIOB->BSRR |= GPIO_BSRR_BS_11;
}
void turn_on_c(void){
	GPIOB->BSRR |= GPIO_BSRR_BS_12;
}
void turn_on_d(void){
	GPIOB->BSRR |= GPIO_BSRR_BS_13;
}
void turn_on_e(void){
	GPIOB->BSRR |= GPIO_BSRR_BS_14;
}
void turn_on_f(void){
	GPIOB->BSRR |= GPIO_BSRR_BS_15;
}

void turn_off_a(void){
	GPIOB->BSRR |= GPIO_BSRR_BR_10;
}
void turn_off_b(void){
	GPIOB->BSRR |= GPIO_BSRR_BR_11;
}
void turn_off_c(void){
	GPIOB->BSRR |= GPIO_BSRR_BR_12;
}
void turn_off_d(void){
	GPIOB->BSRR |= GPIO_BSRR_BR_13;
}
void turn_off_e(void){
	GPIOB->BSRR |= GPIO_BSRR_BR_14;
}
void turn_off_f(void){
	GPIOB->BSRR |= GPIO_BSRR_BR_15;
}

void turn_off_all(void){
	turn_off_a();
	turn_off_b();
	turn_off_c();
	turn_off_d();
	turn_off_e();
	turn_off_f();
}