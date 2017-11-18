#ifndef SPARK
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "serial.h"
#include "stepper_control.h"

int do_new_game(char *params) {
	return do_calibrate(params);
}

int do_end_turn(char *params) {
    return -1;
}

static char get_color(char *flags) {
	if (strchr(flags, 'w')) {
		return 'w';
	} else if (strchr(flags, 'b')) {
		return 'b';
	} else {
		return 0;
	}
}

/*
 *  moves pieces around board, in general the format is:
 *	move,flags,extra_move
 *
 *  example move formats accepted:
 *  * regular move: e2e4
 *  * castling: e1g1,k,h1f1
 *  * capture: e2e4,c
 *  * en passant: b4a3,e,a4
 *  * promotion: h7h8,p,q
 *  * promotion-capture: h7h8,pc,q
 */
int do_move_piece(char *params) {
	#define MAX_PARAMS 3
	int ret = -1;
	char x, y, color;
	grid_t graveyard;
	char *p_arr[MAX_PARAMS] = {NULL};
	int num_params;
	num_params = parse_params(params, p_arr, MAX_PARAMS);
	switch (num_params) {
		case 3:
			color = get_color(p_arr[1]);
			if (strchr(p_arr[1], 'k')) { // castling
				LOG_TRACE("castling move, moving %s...", p_arr[0]);
				uci_move(p_arr[0]); // move king
				LOG_TRACE("moving %s...", p_arr[2]);
				uci_move(p_arr[2]); // move rook
			} else if (strchr(p_arr[1], 'e')) { // en passant
				LOG_TRACE("en passant move, moving %s to graveyard", p_arr[2]);
				// move passed piece to graveyard
				SET_COORDS(x, y, p_arr[2]);
				MOVE_PIECE_TO_GRAVEYARD(x, y, graveyard, color);
				uci_move(p_arr[0]);
			} else if (strchr(p_arr[1], 'p')) { // promotion
				LOG_TRACE("promoting %.2s to a %s at %s", p_arr[0], p_arr[2], p_arr[0]+2);
				// move pawn at 'from' to graveyard
				SET_COORDS(x, y, p_arr[0]);
				MOVE_PIECE_TO_GRAVEYARD(x, y, graveyard, color);
				SET_COORDS(x, y, p_arr[0]+2);
				if (strchr(p_arr[1], 'c')) {
					// move captured piece at 'to' to graveyard
					LOG_TRACE("capturing %s as well", p_arr[0]+2);
					MOVE_PIECE_TO_GRAVEYARD(x, y, graveyard, color);
				}
				// move promoted piece to 'to'
			}
			break;
		case 2: // capture
			// move captured piece at 'to' to graveyard
			LOG_TRACE("capture move, moving %s to graveyard", p_arr[0]+2);
			SET_COORDS(x, y, p_arr[0]+2);
			MOVE_PIECE_TO_GRAVEYARD(x, y, graveyard, get_color(p_arr[1]));
			/* FALLTHROUGH */
		case 1: // normal move
			if (strlen(p_arr[0]) == 2) { // debug move
				int16_t dbg_x = p_arr[0][0]-'0';
				int16_t dbg_y = p_arr[0][1]-'0';
				LOG_TRACE("moving to (%d,%d)", dbg_x, dbg_y);
				debug_move(dbg_x, dbg_y);
			} else {
				LOG_TRACE("moving piece at %s", p_arr[0]);
				ret = (uci_move(p_arr[0]) >= 0) ? 0 : -1;
			}
			break;
		default:
			break;
	}
	
	return ret;
}

int do_promote(char *params) {
	return -1;
}

int do_calibrate(char *params) {
	calibrate();
	SEND_CMD_P(CMD_STATUS, "%d", OKAY);
    return 0;
}

int do_end_game(char *params) {
    return -1;
}

int do_send_log(char *params) {
	return -1;
}

int do_scan_wifi(char *params) {
	do_calibrate(params);
    debug_squares();
	return 0;
}

int do_set_wifi(char *params) {
	magnet_on();
    return 0;
}
#endif
