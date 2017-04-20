/*
 *	Simple fixed-size queue data structure and functions
 */
 
#include "include/queue.h"
#ifdef UNIT_TEST
#include <stdio.h>
#else
#include "stm32f0xx_hal.h"
#endif

void init(step_queue_t *q) {
	int i;
	q->head = 0;
	q->end = 0;
	for (i = 0; i < SIZE; i++) {
		q->queue[i].x_steps = 0;
		q->queue[i].y_steps = 0;
	}
}
 
 /*
  *
  */
int add(step_queue_t *q, int x, int y) {
	 if(!full(q)) { // queue not full
		 q->queue[q->end++].x_steps = x;
		 q->queue[q->end].y_steps = y;
		 
		 if (q->end == SIZE) { // wrap
			 q->end = 0;
		 }
		 
		 return 1;
	 } else {
		return 0;
	 }
 }
 
steps_t rm(step_queue_t *q) {
	steps_t current = {0,0};
	if(!empty(q)) { // queue not empty
		current = q->queue[q->head++];
		q->queue[q->head].x_steps = 0;
		q->queue[q->head].y_steps = 0;
		
		if (q->head == SIZE) { // wrap
			q->head = 0;
		}
	} 
	return current;
}
 
int empty(step_queue_t *q) {
	 if (q->queue[q->head].x_steps == 0 && q->queue[q->head].x_steps == 0)
		 return 1;
	 else
		 return 0;
 }
 
int full(step_queue_t *q) {
	 if (q->head == q->end && (q->queue[q->head].x_steps != 0 || q->queue[q->head].x_steps != 0))
		 return 1;
	 else
		 return 0;
 }
