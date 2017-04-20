#include "include/stepper.h"
#include "include/gpio.h"

// globals
int x_step; // current steps to take
int y_step;
int x_pos;  // global position on axis
int y_pos;

void step_init(void) {
	step_stop(X);
	step_stop(Y);
}

void step_stop(int axis) {
	if (axis == X) {
		SLEEP(X);
		x_step = OFF;
		set_dir(X, 0);
		X_PWM = 0;
	} else if (axis == Y) {
		SLEEP(Y);
		y_step = OFF;
		set_dir(Y, 0);
		Y_PWM = 0;
	}
}

void step(void) {
	if (x_step > 0) {
		x_step--;
	} else if (x_step == 0) {
		step_stop(X);
	} 
	
	if (y_step > 0) {
		y_step--;
	} else if (y_step == 0){
		step_stop(Y);
	}
	TIM2->SR &= ~(TIM_SR_UIF);
}

/*
	Rotate n steps
*/
void stepn(int axis, int n, int dir) {
	// don't step if currently stepping
	if (axis == X && x_step == OFF) {
		x_step = n-1;
		set_dir(X, dir);
		X_PWM = DUTY_CYCLE;
		WAKE(X);
	}
	else if (axis == Y && y_step == OFF) {
		y_step = n-1;
		set_dir(Y, dir);
		Y_PWM = DUTY_CYCLE;
		WAKE(Y);
	}	
}

void step_mm(int axis, int mm, int dir) {
	static int err = 0;
	int adjust = 0;
	int steps = 0;
	
	// handle slight step error
	if (err == 3) {
		adjust = 1;
		err = 0;
	}
	
	steps = (mm*STEPS_PER_MM_16)-adjust;
	stepn(axis, steps, dir);
	err++;
}

// squares are 2" or ~51mm
void step_squares(int axis, int n) {
	int dir;
	if (n > 0) {
		dir = CW;
	} else if (n < 0) {
		dir = CCW;
	} else {
		return;
	}
	step_mm(axis, SQUARE_WIDTH*n, dir);
}

void set_dir(int axis, int dir) {
	if (axis == X) {
		gpio_write_reg16(&(GPIOC->ODR), X_DIR, dir);
	} else if (axis == Y) {
		gpio_write_reg16(&(GPIOC->ODR), Y_DIR, dir);
	}
}

int get_steps(int axis) {
	if (axis == X) {
		return x_step;
	} else {
		return y_step;
	}
}

int get_pos(int axis) {
	if (axis == X) {
		return x_pos;
	} else {
		return y_pos;
	}
}
