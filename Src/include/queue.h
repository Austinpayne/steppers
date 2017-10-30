#ifndef __QUEUE_H_
#define __QUEUE_H_

#define SIZE 128 // proabably needs to be bigger
#define INCREMENT(q,s) (q)->s = ((q)->s == &(q->queue[SIZE-1])) ? (q)->queue : (q)->s+1

typedef int (*done_func)(void);

// steps tuple
typedef struct {
	int x;
	int y;
	done_func done; // function to run when done
} tuple_t;

// queue of steps
typedef struct {
	tuple_t *head;
	tuple_t *end;
	tuple_t queue[SIZE];
} tuple_queue_t;

void init(tuple_queue_t *q);
unsigned char add(tuple_queue_t *q, int x, int y, done_func d);
unsigned char is_empty(tuple_queue_t *q);
unsigned char is_full(tuple_queue_t *q);
void clear_queue(tuple_queue_t *q);
tuple_t  rm(tuple_queue_t *q);

#endif /* __QUEUE_H_ */
