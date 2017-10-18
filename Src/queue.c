/*
 *	Simple fixed-size queue data structure and functions
 */
 
#include "include/queue.h"
#ifdef UNIT_TEST
#include <stdio.h>
#else
#include "stm32f0xx_hal.h"
#endif

/*
 *	initialize queue
 */
void init(tuple_queue_t *q) {
	int i;
	q->head = q->end = q->queue;
	for (i = 0; i < SIZE; i++) {
		q->queue[i].x = 0;
		q->queue[i].y = 0;
		q->queue[i].magnet_bitmap = 0;
	}
}
 
/*
 *	add tuple (x, y) to end of queue 
 */
int add(tuple_queue_t *q, int x, int y, char magnet_bitmap) {
	if(!is_full(q) && ( // queue not full and
		 (x != 0 && y != 0) || // x,y are both non-zero
		 (x == 0 && y != 0) || // or x is zero but y is non-zero
		 (x !=0  && y ==0))) { // or x is non-zero by y is zero
		 q->end->x = x;
		 q->end->y = y;
		 q->end->magnet_bitmap = magnet_bitmap;

		 q->end = (q->end == &(q->queue[SIZE-1])) ? q->queue : q->end+1;
		 
		 return 1;
	 } else {
		return 0;
	 }
}
 
/*
 *	remove and return tuple at head of queue
 */
tuple_t rm(tuple_queue_t *q) {
	tuple_t current = {0,0}; // empty slot represented by 0,0
	if(!is_empty(q)) { // queue not empty
		current = *(q->head);
		q->head->x = 0;
		q->head->y = 0;
		q->head->magnet_bitmap = 0;

		q->head = (q->head == &(q->queue[SIZE-1])) ? q->queue : q->head+1;
	} 
	return current;
}

/*
 *	check if queue is empty
 */
int is_empty(tuple_queue_t *q) {
	 if (q->head == q->end && q->head->x == 0 && q->head->y == 0)
		 return 1;
	 else
		 return 0;
}
 
/*
 *	check if queue is full
 */
int is_full(tuple_queue_t *q) {
	 if (q->head == q->end && (q->head->x != 0 || q->head->y != 0))
		 return 1;
	 else
		 return 0;
}

/*
 *	remove all tuples from queue
 */
void clear_queue(tuple_queue_t *q) {
	while (!is_empty(q)) {
		rm(q);
	}
}
