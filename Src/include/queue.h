#ifndef __QUEUE_H_
#define __QUEUE_H_

#define SIZE 25

// steps tuple
typedef struct {
	int x;
	int y;
} tuple_t;

// queue of steps
typedef struct {
	int head;
	int end;
	tuple_t queue[SIZE];
} tuple_queue_t;

void init(tuple_queue_t *q);
int  add(tuple_queue_t *q, int x, int y);
tuple_t  rm(tuple_queue_t *q);
int  empty(tuple_queue_t *q);
int  full(tuple_queue_t *q);
void clear_queue(tuple_queue_t *q);

#endif /* __QUEUE_H_ */
