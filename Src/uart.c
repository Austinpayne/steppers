#include "stm32f0xx_hal.h"
#include "include/uart.h"
#include "string.h"
#include "include/stepper_control.h"

static char uart_rx_buffer[8];
static int i = 0;

/*
 *	tx one char to serial
 */
static void tx_char(char character) {
	while(1) {
		if (USART1->ISR & USART_ISR_TXE) {
			break;
		}
	}
	USART1->TDR = character;
}

/*
 *	tx string to serial
 */
static void tx_string(const char *string) {
	while (*string != '\0') {
		tx_char(*string);
		string++;
	}
}

int __io_putchar(int ch) {
    while(1) {
		if (USART1->ISR & USART_ISR_TXE) {
			break;
		}
	}
	USART1->TDR = ch;
	return 0;
}

void tx_cmd_char(unsigned char cmd, unsigned char param) {
	tx_char(cmd);
	tx_char(' ');
	tx_char(param);
	tx_char('\n');
}

void tx_cmd(unsigned char cmd, const char *params) {
	tx_char(cmd);
	tx_char(' ');
	tx_string(params);
	tx_char('\n');
}

/*
 *	looking for 4 chars indicating src/dest move:
*		ex: e2e2 = move piece at e2 to e4
 */
void rx_move(void) {
	char temp = USART1->RDR;
	
	if (i < 4) {
		uart_rx_buffer[i] = temp;
		i++;
	}
	
	if (i == 4) {
		uart_rx_buffer[i] = '\0';
		//add_to_queue(50, 0, MAGNET_OFF_OFF);
		//add_to_queue(0, 50, MAGNET_OFF_OFF);
		//add_to_queue(-50, 0, MAGNET_OFF_OFF);
		//add_to_queue(0, -50, MAGNET_OFF_OFF);
		uci_move(uart_rx_buffer);
		i = 0;
		memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
	}
}

/*
void rx_more(void) {
	static char uart_rx_buffer[32];
	static int i = 0;
	char temp = USART1->RDR;
	//tx_char(temp); // for usability with serial terminal
	
	// TODO: add more error checking
	if (temp != 16 && temp != 3) { // ignore control chars that Photon sends
		if (temp == '\r' || temp == '\n') {
			uart_rx_buffer[i++] = '\0'; // terminate string
			// now process
			char *cmd = strtok(uart_rx_buffer, " ");
			if (cmd && strcmp(cmd, "move") == 0) {
				char *coords = NEXT_TOKEN("\n");
				if (coords) {
					if(strchr(coords, 'x')) {
						// uci_capture
					} else if (coords[strlen(coords)-1] > '9') {
						// uci_promo
					} else { // move
						uci_move(coords);
					}
				}
			}
			i = 0; // clear buffer
			memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
		} else {
			uart_rx_buffer[i++] = temp;
		}
	}
}
*/

/*
 *	read in serial data, char by char,
 *  until finding a move command.
 *
 *	looking for 'move' followed by:
 *		'e2e4'  = pawn move
 *		'e2e4X' = pawn promotion to X
 *		'Xe2e4' = piece move where X is:
 *			K = king
 *			Q = queen
 *			R = rook
 *			B = bishop
 *			N = knight
 *		'Xd3xd7' = X captures piece on d7
 *		'exd6' = en passant (only indicate pawns file of depature and dest square)
 */
//void USART1_IRQHandler(void) {
//	rx_move();
//}
