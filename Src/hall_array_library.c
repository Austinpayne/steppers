#include "hall_array_library.h"

//uint16_t biases[8][8];
//uint16_t board_state[8][8];

static board_buffer current_biases;
static board_buffer current_state;
static  board_buffer magnet_pos;

const int16_t THRESHOLD = 100;

void swap(char* a, char* b)
{
 int temp = *a;
 *a = *b;
 *b = temp;
}

/* A utility function to reverse a string  */
void reverse(char str[], int length){
    int start = 0;
    int end = length -1;
    while (start < end){
        swap((str+start), (str+end));
        start++;
        end--;
    }
}
board_buffer scan_bools(board_buffer* state, board_buffer* biases){
	unsigned row = 0, col = 0;
	board_buffer bools;
	for(int i = 0; i < 8; i++){
		for(int j = 0; j < 8; j++){
			int16_t current_state_val = state->buffer[i][j];
			int16_t current_bias_val = biases->buffer[i][j];
			int16_t current_diff = current_state_val - current_bias_val;
			if(current_diff > THRESHOLD){
				bools.buffer[i][j] = 1;
			}else{
				bools.buffer[i][j] = 0;
			}
		}
	}
		return bools;
}
// Implementation of itoa()
char* itoa(int num, char* str, int base){
    int i = 0;
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
        int rem = num % base;
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

void initialize_biases(){
	int x = 0;
	board_buffer temp;
	temp = scan_array(&current_biases);
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
float adcval_tovolt(uint16_t adc_val){
	float val = 0;
	float temp = 3.3/4096;
	val =((float)adc_val)*temp;
	return temp;
}
void transmit_voltage_usart(uint16_t val){
	
	uint16_t mask = 0xff00;
	uint16_t tx = 0;
	uint8_t transmit = 0;
	for(unsigned i = 0; i < 2; i ++){
		tx = mask & val;
		mask = mask >> 8;
		tx  = tx >> (8 * (1 - i));
		transmit = (uint8_t) tx;
		transmit_char_usart((char)(transmit + '0'));
	}
	transmit_char_usart('\r');
	transmit_char_usart('\n');
}

board_buffer scan_array(board_buffer* board){
	//uint16_t array_values[64][64];
	board_buffer state;
	uint16_t row = 0, col = 0;
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
			HAL_Delay(8);
			int16_t reading = take_reading();
			state.buffer[row][col] = reading;
			
			int string_int = reading & 0x0fff;
		}
	}
	return state;
}
int16_t take_reading(void){
	int16_t result = 0;
	int16_t reading = 0;
	for(unsigned j = 0; j < 2; j++){
		while((ADC1->SR & ADC_SR_EOC) == 0){
		//implement time-out
		}
		reading = ADC1->DR & 0xfff;
		reading = 0;
	}
	for(unsigned i = 0; i < 7; i++){
		while((ADC1->SR & ADC_SR_EOC) == 0){
		//implement time-out
		}
		reading = ADC1->DR & 0xfff;
		result += reading;
	}
	result = result / 7;
	return result;
}

void print_board(board_buffer board){
	for(int i = 7; i >= 0; i--){// i is the row
		for(int j = 0; j < 8; j++){// j is the col
			uint16_t string_int = board.buffer[i][j];
			char buffer [sizeof(uint16_t)*8+1];
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
	
	//uint16_t biases[64][64];
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
	int16_t reading = 0;
	//look at mux 0,0,0,0,0,0 - A1
	GPIOB->BSRR |= GPIO_BSRR_BR_10;
	GPIOB->BSRR |= GPIO_BSRR_BR_11;
	GPIOB->BSRR |= GPIO_BSRR_BR_12;
	GPIOB->BSRR |= GPIO_BSRR_BR_13;
	GPIOB->BSRR |= GPIO_BSRR_BR_14;
	GPIOB->BSRR |= GPIO_BSRR_BR_15;
	
	//GPIOB->BSRR |= GPIO_BSRR_BS_12;

	ADC1->CR2 |= ADC_CR2_SWSTART;//turn on adc
	board_buffer temp;
//	current_biases = scan_array(temp);
	current_biases = scan_array(&current_biases);
	current_state = scan_array(&current_state);
	current_biases = scan_array(&current_biases);


  while (1)
  {
		while((ADC1->SR & ADC_SR_EOC) == 0){
			//implement time-out
		}
//		board_buffer temp;
		//current_biases = scan_array(temp);
		//current_state = scan_array(temp);
		//magnet_pos = scan_bools();
		magnet_pos = check_three_boards();
		//print_board(current_state);
		//transmit_char_usart('\r');
		//transmit_char_usart('\n');
		
		print_board(magnet_pos);
		/*PLACE PIECES NOW DISPLAY ON DISPLAY*/
		/*REPORT TO SERVER*/	
		transmit_char_usart('\r');
		transmit_char_usart('\n');
		HAL_Delay(10);
  }
  /* USER CODE END 3 */
}
board_buffer check_three_boards(){
	board_buffer temp;
	//temp = scan_array(&current_state);
	board_buffer board_1 = scan_bools(&current_state, &current_biases);
	current_state = scan_array(&current_state);
	board_buffer board_2 = scan_bools(&current_state, &current_biases);
	current_state = scan_array(&current_state);
	board_buffer board_3 = scan_bools(&current_state, &current_biases);
	
	for(unsigned i = 0; i < 8; i++){
		for(unsigned j = 0; j < 8; j++){
			if(board_1.buffer[i][j] && board_2.buffer[i][j] && board_3.buffer[i][j]){
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