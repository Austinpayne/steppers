#include "include/stepper.h"
#include "include/gpio.h"

// globals
int x_step; // current steps to take
int y_step;
int x_pos;  // global position on axis (in steps)
int y_pos;
int x_dir;
int y_dir;

void step_init(void) {
	step_stop(X);
	step_stop(Y);
	x_pos = 0;
	y_pos = 0;
	x_dir = 1;
	y_dir = 1;
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
		x_pos += x_dir;
	} else if (x_step == 0) {
		step_stop(X);
	} 
	
	if (y_step > 0) {
		y_step--;
		y_pos += y_dir;
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

void step_mm(int axis, int mm) {
	int dir;
	if (mm > 0) {
		dir = CW;
	} else if (mm < 0) {
		dir = CCW;
	} else {
		return;
	}
	mm = abs(mm);
	// handle slight step error
	int adjust = mm/ERR_PER_STEP_16;
	int steps = (mm*STEPS_PER_MM_16)-adjust;
	stepn(axis, steps, dir);
}

void set_dir(int axis, int dir) {
	if (axis == X) {
		if (dir == CW) {
			x_dir = 1;
		} else {
			x_dir = -1;
		}
		gpio_write_reg16(&(GPIOC->ODR), X_DIR, dir);
	} else if (axis == Y) {
		if (dir == CW) {
			y_dir = 1;
		} else {
			y_dir = -1;
		}
		gpio_write_reg16(&(GPIOC->ODR), Y_DIR, dir);
	}
}

int get_steps(int axis) {
	return (axis == X ? x_step : y_step);
}

int get_pos(int axis) {
	return (axis == X ? x_pos : y_pos);
}

/*
 *	Absolute value function
 */
int abs(int val) {
	return (val < 0 ? -val : val);
}
