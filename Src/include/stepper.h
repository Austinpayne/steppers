#ifndef __STEPPER_H_
#define __STEPPER_H_

// stepper PWM
// frequency = (8MHz/PSC/ARR) *approximatly*
#define PRESCALE 50
#define AUTO_RELOAD 20
#define DUTY_CYCLE 1

// step & gear constants
#define GEAR_C             30     // approx gear circumference, in mm
#define STEP_ANGLE         0.0314 // radians, approx 1.8*
#define GEAR_RADIUS        4.75   // mm
#define LINEAR_DISTANCE    0.15   // mm, STEP_ANGLE*GEAR_RADIUS
#define STEPS_PER_REV_FULL 200    // full step
#define STEPS_PER_REV_16   3200   // 1/16 microstep
#define STEPS_PER_MM_FULL  7      //  (STEPS_PER_REV_FULL*MICROSTEP)/GEAR_C
#define STEPS_PER_MM_16    100    // 1/16 microstep
#define ERR_PER_STEP_16    0      // 1/16 step (1 = No Error)
// 1 square == ~51mm == ~5457 steps (@ 1/16 microstep)

// axis
#define X 0
#define Y 1
#define OFF -1
#define CW   0 // + direction
#define CCW  1 // - direction

// axis pins (GPIOC)
#define X_DIR   10
#define X_RESET 12
#define X_CAL   4 // input 
#define X_SLEEP 6
#define X_STEP  8
#define X_PWM   TIM3->CCR3

#define Y_DIR   11
#define Y_RESET 3
#define Y_CAL   5 // input
#define Y_SLEEP 7
#define Y_STEP  9
#define Y_PWM   TIM3->CCR4

// helper macros
#define SLEEP(axis)     (GPIOC->ODR &= (axis == X) ? ~(1 << X_SLEEP) : ~(1 << Y_SLEEP))
#define WAKE(axis)      (GPIOC->ODR |= (axis == X) ?  (1 << X_SLEEP) :  (1 << Y_SLEEP))
#define MM_TO_STEPS(mm) ((mm)*STEPS_PER_MM_16)
#define STEPS_TO_MM(s)  ((s)/STEPS_PER_MM_16)

// func definitions
void step_reset(void);
void stop_axis(int axis);
void stop_stepping(void);
void step(void);
void stepn(int axis, int n, int dir);
void step_mm(int axis, int mm);
void set_dir(int axis, int dir);
int  axis_stepping(int axis);
unsigned char stepping(void);
int  get_pos(int axis);
int  abs(int val);

#endif /* __STEPPER_H_ */
