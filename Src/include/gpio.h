#ifndef __GPIO_H_
#define __GPIO_H_

#include "stm32f0xx_hal.h"

void gpio_output_init(GPIO_TypeDef * port, uint32_t pin);
void gpio_alternate_init(GPIO_TypeDef * port, uint32_t pin);
void gpio_input_init(GPIO_TypeDef * port, uint32_t pin);
void gpio_write_reg32(__IO uint32_t * reg, uint32_t pin, uint32_t bits);
void gpio_write_reg16(__IO uint32_t * reg, uint32_t pin, uint32_t bits);
void gpio_toggle_reg16(__IO uint32_t * reg, uint32_t pin);

#endif /* __GPIO_H_ */
