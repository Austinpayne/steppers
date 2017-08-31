#include "include/stepper.h"
#include "include/gpio.h"

// globals
int x_step; // current steps to take
int y_step;
int x_pos;  // global position on axis (in steps)
int y_pos;
int x_dir;  // unit to add/subtract (for position, either +1 or -1)
int y_dir;

/*
 *	initialize step counters and position
 */
void step_init(void) {
	step_stop(X);
	step_stop(Y);
	x_pos = 0;
	y_pos = 0;
	x_dir = 1;
	y_dir = 1;
}

/*
 *	stops the axis by turning off axis PWM,
 *  and putting the motor driver into sleep mode
 */
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

/*
 *	decrements step by one, when
 *  steps == 0, stops stepping
 */
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

/*
 *	set internal step count
 *  set axis direction by passing + or - mm
 *  + for clockwise, - for counter-clockwise
 */
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
	int steps = mm_to_steps(mm)-adjust;
	stepn(axis, steps, dir);
}

/*
 *	Set direction of axis
 */
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

/*
 *	get current axis steps
 */
int get_steps(int axis) {
	return (axis == X ? x_step : y_step);
}

/*
 *	return current axis position, in mm
 */
int get_pos(int axis) {
	return (axis == X ? steps_to_mm(x_pos) : steps_to_mm(y_pos));
}

/*
 *	mm = steps/STEPS_PER_MM
 */
int steps_to_mm(int steps) {
	return steps/STEPS_PER_MM_16;
}

/*
 *	steps = mm*STEPS_PER_MM
 */
int mm_to_steps(int mm) {
	return mm*STEPS_PER_MM_16;
}

/*
 *	Absolute value function
 */
int abs(int val) {
	return (val < 0 ? -val : val);
}
