#include "gpio.h"

/**
 *	Initializes the specified GPIO port and pin to be 
 *	an output with the following settings:
 *		- push-pull output, low speed, no pull-up/down resistor
 */
void gpio_output_init(GPIO_TypeDef * port, uint32_t pin) {
	gpio_write_reg32(&(port->MODER), pin, 1); // mode (output 01)
	gpio_write_reg16(&(port->OTYPER), pin, 0); // type (push-pull 0)
	gpio_write_reg32(&(port->OSPEEDR), pin, 0); // speed (low x0)
	gpio_write_reg32(&(port->PUPDR), pin, 0); // resistor (none 00)
}

void gpio_alternate_init(GPIO_TypeDef * port, uint32_t pin) {
	gpio_write_reg32(&(port->MODER), pin, 2); // mode (alternate 10)
	gpio_write_reg16(&(port->OTYPER), pin, 0); // type (push-pull 0)
	gpio_write_reg32(&(port->OSPEEDR), pin, 0); // speed (low x0)
	gpio_write_reg32(&(port->PUPDR), pin, 0); // resistor (none 00)
}

/**
 *	Initializes the specified GPIO port and pin to be 
 *	an input with the following settings:
 *		- low speed, pull-up resistor
 */
void gpio_input_init(GPIO_TypeDef * port, uint32_t pin) {
	gpio_write_reg32(&(port->MODER), pin, 0); // mode (input 00)
	gpio_write_reg32(&(port->OSPEEDR), pin, 0); // speed (low x0)
	gpio_write_reg32(&(port->PUPDR), pin, 1); // resistor (pull-up 01)
}

/**
 *	Writes/clears the given bits at the GPIO pin (32 bit register).
 *	Note that the 32 bit refers to the fact that the register uses
 *	all 32 bits, where each GPIO pin occupies 2 bits
 */
void gpio_write_reg32(__IO uint32_t * reg, uint32_t pin, uint32_t bits) {
	if (bits)
		*reg |= (bits << (pin + pin));
	else // clear
		*reg &= ~(3 << (pin + pin));
}


/**
 *	Writes/clears the given bits at the GPIO pin (16 bit register).
 *	Note that the 16 bit refers to the fact that the register uses
 *	only the bottom 16 bits, where each GPIO pin occupies 1 bit.
 */
void gpio_write_reg16(__IO uint32_t * reg, uint32_t pin, uint32_t bits) {
	if (bits)
		*reg |= (bits << pin);
	else // clear
		*reg &= ~(1 << pin);
}

/**
 *	Toggles the GPIO pin (16 bit register).
 *	Note that the 16 bit refers to the fact that the register uses
 *	only the bottom 16 bits, where each GPIO pin occupies 1 bit.
 */
void gpio_toggle_reg16(__IO uint32_t * reg, uint32_t pin) {
	*reg ^= (1 << pin);
}
