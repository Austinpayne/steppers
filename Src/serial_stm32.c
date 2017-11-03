#ifndef SPARK
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "serial.h"
#include "stepper_control.h"

int do_new_game(char *params) {
	return do_calibrate(params);
}

int do_end_turn(char *params) {
    return -1;
}

// only for testing, captures a move command from usb serial
// and forwards it to motor driver
int do_move_piece(char *params) {
	LOG_TRACE("got move: %s", params);
	if (params && uci_move(params) >= 0) {
		LOG_TRACE("move succeded");
		return 0;
	}
	LOG_TRACE("move failed");
    return -1;
}

int do_promote(char *params) {
    return -1;
}

int do_calibrate(char *params) {
	LOG_TRACE("calibrating...");
	calibrate();
    return 0;
}

int do_end_game(char *params) {
    return -1;
}

int do_scan_wifi(char *params) {
    return -1;
}

int do_set_wifi(char *params) {
    return -1;
}
#endif
