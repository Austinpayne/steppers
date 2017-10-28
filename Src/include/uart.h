#ifndef __UART_H_
#define __UART_H_

#define NEXT_TOKEN(d) (strtok(NULL, d))

#define CMD_STATUS    0x0
#define CMD_NEW_GAME  0x1
#define CMD_END_TURN  0x2
#define CMD_PROMOTE   0x3
#define CMD_END_GAME  0x5
#define CMD_SCAN_WIFI 0x6
#define CMD_SET_WIFI  0x7

#define OK 0
#define FAIL 1

void tx_cmd_char(unsigned char cmd, unsigned char param);
void tx_cmd(unsigned char cmd, const char *params);

#endif /* __UART_H_ */
