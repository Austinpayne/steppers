/*
 *	Simple fixed-size queue data structure and functions
 */
 
#include "include/queue.h"
#ifdef UNIT_TEST
#include <stdio.h>
#else
#include "stm32f0xx_hal.h"
#endif

void init(tuple_queue_t *q) {
	int i;
	q->head = 0;
	q->end = 0;
	for (i = 0; i < SIZE; i++) {
		q->queue[i].x = 0;
		q->queue[i].y = 0;
	}
}
 
 /*
  *
  */
int add(tuple_queue_t *q, int x, int y) {
	 if(!full(q) && ( // queue not full and
		 (x != 0 && y != 0) || // x,y are both non-zero
		 (x == 0 && y != 0) || // or x is zero but y is non-zero
		 (x !=0  && y ==0))) { // or x is non-zero by y is zero
		 q->queue[q->end].x = x;
		 q->queue[q->end].y = y;
		 q->end++;
		 
		 if (q->end == SIZE) { // wrap
			 q->end = 0;
		 }
		 
		 return 1;
	 } else {
		return 0;
	 }
 }
 
tuple_t rm(tuple_queue_t *q) {
	tuple_t current = {0,0}; // empty slot represented by 0,0
	if(!empty(q)) { // queue not empty
		current = q->queue[q->head];
		q->queue[q->head].x = 0;
		q->queue[q->head].y = 0;
		q->head++;
		
		if (q->head == SIZE) { // wrap
			q->head = 0;
		}
	} 
	return current;
}
 
int empty(tuple_queue_t *q) {
	 if (q->head == q->end && q->queue[q->head].x == 0 && q->queue[q->head].y == 0)
		 return 1;
	 else
		 return 0;
}
 
int full(tuple_queue_t *q) {
	 if (q->head == q->end && (q->queue[q->head].x != 0 || q->queue[q->head].y != 0))
		 return 1;
	 else
		 return 0;
}

void clear_queue(tuple_queue_t *q) {
	while (!empty(q)) {
		rm(q);
	}
}
