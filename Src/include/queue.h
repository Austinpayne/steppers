#ifndef __QUEUE_H_
#define __QUEUE_H_

#define SIZE 25

// steps tuple
typedef struct {
	int x_steps;
	int y_steps;
} steps_t;

// queue of steps
typedef struct {
	int head;
	int end;
	steps_t queue[SIZE];
} step_queue_t;

void init(step_queue_t *q);
int  add(step_queue_t *q, int x, int y);
steps_t  rm(step_queue_t *q);
int  empty(step_queue_t *q);
int  full(step_queue_t *q);

#endif /* __QUEUE_H_ */
