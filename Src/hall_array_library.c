#include "serial.h"
#include "string.h"
#include "hall_array_library.h"

volatile board_buffer* cur_state;
volatile board_buffer* current_biases;
//char * move_parse = "abcdefgh";
char* move_parse = "hgfedcba";
//volatile board_buffer* mag_pos;

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

void scan_bools(board_buffer* bools) {
	for(int16_t i = 0; i < 8; i++){
		for(int16_t j = 0; j < 8; j++){
			int16_t current_bias_val = current_biases->buffer[i][j];
			int16_t cur_state_val = cur_state->buffer[i][j];
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

float adcval_tovolt(int16_t adc_val){
	float temp = 3.3/4096;
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

void scan_array(volatile board_buffer* buf){
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
			//for(int32_t k = 0; k < 10000; k++){
			//	__nop();
			//}
			HAL_Delay(15);
 			int16_t reading = take_reading();
			reading &= 0x0fff;
			buf->buffer[row][col] = reading;
		}
	}
	return;
}
/*void rebias_chessboard(){
	
}*/
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
	for(unsigned i = 0; i < 7; i++){
		while((ADC1->ISR & ADC_ISR_EOC) == 0){
		//implement time-out
		}
		reading = ADC1->DR & 0xfff;
		result += reading;
	}
	result = result / 7;
	return result;
}

void print_board(board_buffer* board){
	#define STR_BUFF_SIZE 32
	char row_str[STR_BUFF_SIZE];// = {0};
	int len = 0;
	LOG_TRACE("");
	for(int16_t i = 7; i >= 0; i--){// i is the row
		for(int16_t j = 0; j < 8; j++){// j is the col
			len += snprintf(row_str+len, STR_BUFF_SIZE-len, "%d ", board->buffer[i][j]);
			if(j == 7){
				LOG_TRACE("%s", row_str);
			}
		}
		memset(row_str, 0, STR_BUFF_SIZE);
		len = 0;
	}
}

void init_board(void) {
	cur_state = malloc(sizeof(board_buffer));
	current_biases = malloc(sizeof(board_buffer));
	//mag_pos = malloc(sizeof(mag_pos));
	
	turn_off_all();

	ADC1->CR |= ADC_CR_ADSTART;//turn on adc
	//scan_array(current_biases);
	scan_array(cur_state);
	scan_array(current_biases);
	scan_array(current_biases);
	scan_array(current_biases);
	
}

void get_board_state(void) {
	board_buffer board_1;//, board_2, board_3;
	zero_out_board(&board_1);
	//zero_out_board(&board_2);
	//zero_out_board(&board_3);
	//check_three_boards(&board_1);
	print_board(&board_1);
	return ;
}

void pseudo_main(void){
  board_buffer board_state1, board_state2;
  while (1)
  {
		while((ADC1->ISR & ADC_ISR_EOC) == 0){
			//implement time-out
		}
		board_buffer board_1;//, board_2, board_3;
		zero_out_board(&board_1);
		//zero_out_board(&board_2);
		//zero_out_board(&board_3);
		//zero_out_board(&temp);
		//current_biases = scan_array(temp);
		//scan_array(&cur_state);
		//magnet_pos = scan_bools();
		//board_state1 = check_three_boards(&board_1, &board_2, &board_3);
		print_board(&board_state1);
		HAL_Delay(1000);
		HAL_Delay(1000);
		
		//board_buffer board_state3 = check_three_boards(&board_1, &board_2, &board_3);
		HAL_Delay(1000);	
		HAL_Delay(1000);
		HAL_Delay(1000);
		//zero_out_board(&board_1); zero_out_board(&board_2); zero_out_board(&board_3);
		//scan_array(&cur_state);
		//board_state2 = check_three_boards(&board_1, &board_2, &board_3);
		print_board(&board_state2);
		
		move_string move;
		calculate_move(&board_state1, &board_state2, &move);
		LOG_TRACE("hall_move: %s", move.buf);
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
int8_t calculate_move(board_buffer* prev_state, board_buffer* new_state, move_string* move_buf){
	uint8_t p1x = 0, p1y = 0;
	uint8_t p2x = 0, p2y = 0;
	uint8_t found_first_point = 0;
	uint8_t found_second_point = 0;
	uint8_t prev_x = 0, prev_y = 0;
	uint8_t new_x = 0, new_y = 0;
	for(uint8_t j = 0; j < 8; j++){
		for(uint8_t i = 0; i < 8; i++){
			int16_t test = prev_state->buffer[i][j] ^ new_state->buffer[i][j];
			if(test){
				if(!found_first_point){
					p1x = j; p1y = i;
					found_first_point = 1;
				}else{
					p2x = j; p2y = i;
					found_second_point = 1;
				}
			}
		}
	}
	if(new_state->buffer[p1y][p1x] == 1){
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
	
	move_buf->buf[0] = move_parse[prev_y];//prev_x + 'a';
	move_buf->buf[1] = prev_x + '1';
	move_buf->buf[2] = move_parse[new_y];//new_x + 'a';
	move_buf->buf[3] = new_x + '1';
	move_buf->buf[4] = '\0';
	
	if (found_first_point && found_second_point)
		return 0;
	else
		return -1;
}
void check_three_boards(board_buffer* board_1, board_buffer* board_2) {
	//board_buffer temp;
	scan_array(cur_state);
	scan_bools(board_1);
	scan_array(cur_state);
	scan_bools(board_2);
	for(unsigned i = 0; i < 8; i++){
		for(unsigned j = 0; j < 8; j++){
			if(board_1->buffer[i][j] && board_2->buffer[i][j]){
				board_1->buffer[i][j] = 1;
			}else{
				board_1->buffer[i][j] = 0;
			}
		}
	}
	return;
}

void turn_off_all(void){
	turn_off_a();
	turn_off_b();
	turn_off_c();
	turn_off_d();
	turn_off_e();
	turn_off_f();
}
