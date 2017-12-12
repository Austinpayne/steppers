#ifndef SPARK
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "serial.h"
#include "stepper_control.h"
#include "hall_array_library.h"

board_buffer old_state, new_state;
board_buffer temp1;
uint8_t save_move = 1;
move_string latest_move;

void init_board_state(void) {
	check_three_boards(&old_state, &temp1);
	print_board(&old_state);
}

void update_board_state(uint8_t set_save_move) {
	check_three_boards(&new_state, &temp1);
	print_board(&new_state);
	if (set_save_move) {
		calculate_move(&old_state,&new_state,&latest_move);
	}
	old_state = new_state;
}

int do_new_game(char *params) {
	init_board_state();
	if (count_pieces(&old_state) <= 33) {
		return do_calibrate(params);
	} else {
		LOG_ERR("need 32 pieces on the board");
		SEND_CMD_P(CMD_STATUS, "%d", STATUS_FAIL);
		return -1;
	}
}

int do_end_turn(char *params) {
	check_three_boards(&new_state, &temp1);
	print_board(&new_state);
	int8_t ret = -1;
	if (save_move) {
		ret = calculate_move(&old_state, &new_state, &latest_move);
	} else {
		ret = 0;
	}
	if (ret == 0) {
		SEND_CMD_P(CMD_MOVE_PIECE, "%.4s", latest_move.buf);
		save_move = 1;
		old_state = new_state;
	} else {
		SEND_CMD_P(CMD_MOVE_PIECE, "fail: %.4s", latest_move.buf);
	}
	return 0;
}

int do_capture_castle(char *params) {
	char *p_arr[1] = {NULL};
	int num_params;
	num_params = parse_params(params, p_arr, 1);
	if (num_params > 0) {
		if (strchr(p_arr[0], 'c')) {
			LOG_TRACE("capturing");
			offset_magnet_head();
			update_board_state(0);
		} else if (strchr(p_arr[0], 'k')) {
			LOG_TRACE("castling");
			update_board_state(1);
			save_move = 0;
		}
	}
	SEND_CMD_P(CMD_STATUS, "%d", STATUS_OKAY);
	return 0;
}

static char get_color(char *flags) {
	if (strchr(flags, 'w')) {
		return 'w';
	} else {
		return 'b';
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
				LOG_TRACE("castling move, moving %s and %s", p_arr[0], p_arr[2]);
				uci_move(p_arr[0]); // move king
				ret = uci_move(p_arr[2]); // move rook
			} else if (strchr(p_arr[1], 'e')) { // en passant
				LOG_TRACE("en passant move, moving %s to graveyard", p_arr[2]);
				// move passed piece to graveyard
				SET_COORDS(x, y, p_arr[2]);
				MOVE_PIECE_TO_GRAVEYARD(x, y, graveyard, color);
				ret = uci_move(p_arr[0]);
			} else if (strchr(p_arr[1], 'p')) { // promotion
				LOG_TRACE("promoting %.2s to a %s at %s", p_arr[0], p_arr[2], p_arr[0]+2);
				// move pawn at 'from' to graveyard
				SET_COORDS(x, y, p_arr[0]);
				MOVE_PIECE_TO_GRAVEYARD(x, y, graveyard, color);
				SET_COORDS(x, y, p_arr[0]+2);
				if (strchr(p_arr[1], 'c')) {
					// move captured piece at 'to' to graveyard
					MOVE_PIECE_TO_GRAVEYARD(x, y, graveyard, color);
				}
				// move promoted piece to 'to'
				PROMOTE_TO_SQUARE(x, y, graveyard, color);
				ret = 0;
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
				ret = uci_move(p_arr[0]);
			}
			break;
		default:
			break;
	}
	
	update_board_state(0);
	if (ret == 0) {
		SEND_CMD_P(CMD_STATUS, "%d", STATUS_OKAY);
	} else {
		SEND_CMD_P(CMD_STATUS, "%d", STATUS_FAIL);
	}
	return ret;
}

int do_promote(char *params) {
	return -1;
}

int do_calibrate(char *params) {
	calibrate();
	SEND_CMD_P(CMD_STATUS, "%d", STATUS_OKAY);
    return 0;
}

int do_end_game(char *params) {	
    return -1;
}

int do_send_log(char *params) {
	return -1;
}

int do_retry(char *params) {
	update_board_state(0);
	SEND_CMD_P(CMD_STATUS, "%d", STATUS_OKAY);
    return 0;
}

int do_reset(char *params) {
	NVIC_SystemReset();
	return 0;
}

int do_user_turn(char *params) {
	return -1;
}

int do_debug_cmd(char *params) {
	char *p_arr[1] = {NULL};
	int num_params;
	num_params = parse_params(params, p_arr, 1);
	if (num_params > 0) {
		switch(*(p_arr[0])) {
			case 'p':
				get_board_state();
				break;
			case 'd':
				do_calibrate(params);
				debug_squares();
				break;
			case 'o':
				magnet_on();
				break;
			case 'f':
				magnet_off();
				break;			
		}
	}
	return 0;
}
#endif
