#ifndef __QUEUE_H_
#define __QUEUE_H_

#define SIZE 128 // proabably needs to be bigger
#define INCREMENT(q,s) (q)->s = ((q)->s == &(q->queue[SIZE-1])) ? (q)->queue : (q)->s+1

// steps tuple
typedef struct {
	int x;
	int y;
	char magnet_bitmap; // turn magnet off/on at start and/or end of move
} tuple_t;

// queue of steps
typedef struct {
	tuple_t *head;
	tuple_t *end;
	tuple_t queue[SIZE];
} tuple_queue_t;

void init(tuple_queue_t *q);
int  add(tuple_queue_t *q, int x, int y, char magnet_bitmap);
int  is_empty(tuple_queue_t *q);
int  is_full(tuple_queue_t *q);
void clear_queue(tuple_queue_t *q);
tuple_t  rm(tuple_queue_t *q);

#endif /* __QUEUE_H_ */
