#define SIZE 25

typedef struct {
	int x_steps;
	int y_steps;
} steps_t;

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
