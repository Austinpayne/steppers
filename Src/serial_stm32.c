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
	char x, y;
	grid_t graveyard;
	char *p_arr[MAX_PARAMS] = {NULL};
	int num_params;
	num_params = parse_params(params, p_arr, MAX_PARAMS);
	switch (num_params) {
		case 3:
			if (strchr(p_arr[1], 'k')) { // castling
				LOG_TRACE("castling move, moving %s...", p_arr[0]);
				uci_move(p_arr[0]); // move king
				LOG_TRACE("moving %s...", p_arr[2]);
				uci_move(p_arr[2]); // move rook
			} else if (strchr(p_arr[1], 'e')) { // en passant
				LOG_TRACE("en passant move, moving %s to graveyard", p_arr[2]);
				// move passed piece to graveyard
				SET_COORDS(x, y, p_arr[2]);
				//MOVE_PIECE_TO_GRAVEYARD(x, y, graveyard);
				uci_move(p_arr[0]);
			} else if (strchr(p_arr[1], 'p')) { // promotion
				LOG_TRACE("promoting %.2s to a %s at %s", p_arr[0], p_arr[2], p_arr[0]+2);
				// move pawn at 'from' to graveyard
				SET_COORDS(x, y, p_arr[0]);
				//MOVE_PIECE_TO_GRAVEYARD(x, y, graveyard);
				SET_COORDS(x, y, p_arr[0]+2);
				if (strchr(p_arr[1], 'c')) {
					// move captured piece at 'to' to graveyard
					LOG_TRACE("capturing %s as well", p_arr[0]+2);
					//MOVE_PIECE_TO_GRAVEYARD(x, y, graveyard);
				}
				// move promoted piece to 'to'
			}
			break;
		case 2: // capture
			// move captured piece at 'to' to graveyard
			LOG_TRACE("capture move, moving %s to graveyard", p_arr[0]+2);
			SET_COORDS(x, y, p_arr[0]+2);
			//MOVE_PIECE_TO_GRAVEYARD(x, y, graveyard);
			/* FALLTHROUGH */
		case 1: // normal move
			LOG_TRACE("moving piece at %s", p_arr[0]);
			ret = (uci_move(p_arr[0]) >= 0) ? 0 : -1;
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
