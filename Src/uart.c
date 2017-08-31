#include "stm32f0xx_hal.h"
#include "include/uart.h"
#include "string.h"

/*
 *	tx one char to serial
 */
void tx_char(char character) {
	while(1) {
		if (USART1->ISR & USART_ISR_TXE) {
			break;
		}
	}
	USART1->TDR = character;
}

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
void USART1_IRQHandler(void) {
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