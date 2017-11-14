#include <stdint.h>
#include "stepper.h"
#include "gpio.h"

// globals
static int x_step; // current steps to take
static int y_step;
static int x_pos;  // global position on axis (in steps)
static int y_pos;
static int8_t x_dir;  // unit to add/subtract (for position, either +1 or -1)
static int8_t y_dir;

/*
 *	initialize step counters and position
 *  call this to clear position after calibration
 */
void step_reset(void) {
	stop_stepping();
	x_step = 0;
	y_step = 0;
	x_pos = 0;
	y_pos = 0;
	x_dir = 1;
	y_dir = 1;
}

/*
 *	stops the axis by turning off axis PWM,
 *  and putting the motor driver into sleep mode
 */
void stop_axis(int8_t axis) {
	if (axis == X) {
		X_PWM = 0;
		x_step = OFF;
	} else if (axis == Y) {
		Y_PWM = 0;
		y_step = OFF;
	}
}

void stop_stepping(void) {
	stop_axis(X);
	stop_axis(Y);
}

/*
 *	decrements step by one, when
 *  steps == 0, stops stepping
 */
void step(void) {
	if (x_step > 0) {
		x_step--;
		x_pos += x_dir;
	} else if (x_step <= 0) {
		stop_axis(X);
	}
	
	if (y_step > 0) {
		y_step--;
		y_pos += y_dir;
	} else if (y_step <= 0) {
		stop_axis(Y);
	}
}

/*
	Rotate n steps
*/
void stepn(int8_t axis, int n, int8_t dir) {
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

void step_mm_blocking(int x_mm, int y_mm) {
	step_mm(X, x_mm);
	step_mm(Y, y_mm);
	while (stepping()) {
		HAL_Delay(10);
	}
}

/*
 *	set internal step count
 *  set axis direction by passing + or - mm
 *  + for clockwise, - for counter-clockwise
 */
void step_mm(int8_t axis, int mm) {
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
	int adjust = 0 /*mm/ERR_PER_STEP_16*/;
	int steps = MM_TO_STEPS(mm)-adjust;
	stepn(axis, steps, dir);
}

/*
 *	Set direction of axis
 */
void set_dir(int8_t axis, int8_t dir) {
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
unsigned char axis_stepping(int8_t axis) {
    if (axis == X)
	    return (x_step == OFF ? 0 : 1);
    else
	    return (y_step == OFF ? 0 : 1);
}

unsigned char stepping(void) {
	if (axis_stepping(X) || axis_stepping(Y))
		return 1;
	return 0;
}

/*
 *	return current axis position, in mm
 */
int get_pos(int8_t axis) {
	return (axis == X ? STEPS_TO_MM(x_pos) : STEPS_TO_MM(y_pos));
}

/*
 *	Absolute value function
 */
unsigned int abs(int val) {
	return (val < 0 ? -val : val);
}

/*
 *  update event interrupt, controls continous stepping
 */
void TIM2_IRQHandler(void) {
	step();
	TIM2->SR &= ~(TIM_SR_UIF); // clear interrupt
}
