#ifndef __UART_H_
#define __UART_H_

#define NEXT_TOKEN(d) (strtok(NULL, d))

void tx_char(char character);
void tx_string(char *string);

#endif /* __UART_H_ */
