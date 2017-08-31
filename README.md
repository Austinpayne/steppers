# Dual Axis Stepper Driver

Logic to control the DRV8824/25 motor driver board. The original application is to control a dual axis CNC frame using two 
steppers per axis. However, the code could be adapted for use in any stepper application.

## Src

The main src files are:
- `stepper.c` Stepper driver. Includes functions to step an arbitrary amount of steps or milimeters (given a set gear circumference).
- `queue.c` Simple queue structure that can be used to make sequentially steps. Note that both the x and y axis can step in parallel.
- `stepper_control.c` Functions to control stepper motors for chess robot. Makes use of queue to add moves and converts square Universal Chess Interface coordinates to millimeters/steps.
- `gpio.c` General GPIO control functions STM32 microcontrollers.

## Testing

Run `make test_queue`. This will download the [Unity](http://www.throwtheswitch.org/unity/) testing framework and run the 
the unit tests in `test_queue.c`.
