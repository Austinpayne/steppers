/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

#define X 0
#define Y 1
#define PRESCALE 50
#define AUTO_RELOAD 20
#define DUTY_CYCLE 2
#define STEPS_PER_REV 200 // full step
#define MICROSTEP 16
#define GEAR_C 30 // gear circumference, in mm
#define STEPS_PER_MM (STEPS_PER_REV*MICROSTEP)/GEAR_C
#define STEPS_PER_MM_C 107 // for 1/16 microstep size and gear circumference of 30mm
#define STEP_ANGLE 0.0314 // radians, approx 1.8*
#define GEAR_RADIUS 4.75 // mm
#define LINEAR_DISTANCE 0.15 //mm, STEP_ANGLE*GEAR_RADIUS
#define OFF -1
#define CW   0 // direction
#define CCW  1

// Pins (GPIOC)
#define X_DIR   0
#define X_RESET 2
#define X_CAL   4 // input
#define X_SLEEP 6
#define X_STEP  8

#define Y_DIR   1
#define Y_RESET 3
#define Y_CAL   5 // input
#define Y_SLEEP 7
#define Y_STEP  9




/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void gpio_output_init(GPIO_TypeDef * port, uint32_t pin);
void gpio_input_init(GPIO_TypeDef * port, uint32_t pin);
void gpio_write_reg32(__IO uint32_t * reg, uint32_t pin, uint32_t bits);
void gpio_write_reg16(__IO uint32_t * reg, uint32_t pin, uint32_t bits);
void gpio_toggle_reg16(__IO uint32_t * reg, uint32_t pin);
void step_init(void);
void step(void);
void stepn(int axis, int n);
void step_mm(int axis, int mm);

int stepx;
int stepy;

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void  button_init(void) {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}

void HAL_SYSTICK_Callback(void) {
    static uint32_t debouncer = 0;
    
    debouncer = (debouncer << 1);
    if(GPIOA->IDR & (1 << 0)) {
        debouncer |= 0x1;
    }

    if(debouncer == 0x7FFFFFFF) {
		step_mm(X, 30, CW); // step 30mm, about one rotation using 1/16 microstep
		step_mm(Y, 30, CCW);
    }
    
}

void timer_init(void) {
	RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // PWM
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // for interrupts, mirror PWM
	
	TIM2->PSC   = PRESCALE;
	TIM2->ARR   = AUTO_RELOAD;   
	TIM2->DIER |= TIM_DIER_UIE; // enable timer interrupt
	
	TIM3->PSC = PRESCALE;	// frequency = (8MHz/PSC/ARR) *approximatly*
	TIM3->ARR = AUTO_RELOAD;
	// duty cycle: CCR is % of ARR register (ex. ARR = 10, CCR = 1 => 10% duty cycle)
	TIM3->CCR3 = 0; // PC8
	TIM3->CCR4 = 0; // PC9
	TIM3->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos); // ch 3 capture/compare PWM mode 1
	TIM3->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos); // ch 4 capture/compare PWM mode 1
	//TIM3->CCMR2 |= (7 << TIM_CCMR2_OC3M_Pos); // ch 3 capture/compare PWM mode 2
	//TIM3->CCMR2 |= (7 << TIM_CCMR2_OC4M_Pos); // ch 3 capture/compare PWM mode 2
	TIM3->CCER  |= TIM_CCER_CC3E; // enable ch 3
	TIM3->CCER  |= TIM_CCER_CC4E; // enable ch 4
	
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);
	
	TIM2->CR1 |= TIM_CR1_CEN; // enable timers
	TIM3->CR1 |= TIM_CR1_CEN;
}

void output_init(void) {
	RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
	
	gpio_output_init(GPIOC, X_DIR); // PC0 (x-axis dir)
	gpio_output_init(GPIOC, Y_DIR); // PC1 (y-axis dir)
	
	gpio_output_init(GPIOC, X_SLEEP); // PC6 (x-axis sleep)
	gpio_output_init(GPIOC, Y_SLEEP); // PC7 (y-axis sleep)
	
	// wire PWM to LED to verify output (use PC8 or PC9 to capture PWM output)
	gpio_alternate_init(GPIOC, X_STEP); // PC8 (x-axis step)
	GPIOC->AFR[1] &= ~(GPIO_AFRH_AFRH0);
	gpio_alternate_init(GPIOC, Y_STEP); // PC9 (y-axis step)
	GPIOC->AFR[1] &= ~(GPIO_AFRH_AFRH1);
}

void cal_init(void) {
	RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
	
	gpio_output_init(GPIOC, X_RESET); // PC2 (x-axis reset)
	gpio_output_init(GPIOC, Y_RESET); // PC3 (y-axis reset)
	
	gpio_input_init(GPIOC, X_CAL);
	gpio_input_init(GPIOC, Y_CAL);
	
	// enable cal switch interrupts
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PC; // multiplex PC4 to EXTI4
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PC; // multiplex PC5 to EXTI5
	EXTI->IMR |= (EXTI_IMR_IM4 | EXTI_IMR_IM5); // unmask EXTI4 & EXTI5
	EXTI->RTSR |= (EXTI_RTSR_RT4 | EXTI_RTSR_RT5); // trigger on rising edge
	NVIC_EnableIRQ(EXTI4_15_IRQn); // enable interrupt in NVIC
	NVIC_SetPriority(EXTI4_15_IRQn, 1);
	
	// TODO: move axis until they hit cal switches then clear counters
	
	gpio_write_reg16(&(GPIOC->ODR), X_RESET, 1); // nRESET
	gpio_write_reg16(&(GPIOC->ODR), Y_RESET, 1);
	
}

/* USER CODE END 0 */

int main(void)
{
	// LEDS on PC8, PC9, PC6, PC7 DON'T USE THESE PINS FOR TIMERS
	// Use TIM2_CH2 (PA1) and TIM3_CH4 (PB1)
	HAL_Init();
	SystemClock_Config();
	
	button_init();
	timer_init();
	output_init();
	cal_init();
	step_init();
	
  while (1)
  {
	  __WFI();
  }
}

void EXTI4_15_IRQHandler(void) {
	if (GPIOC->IDR & (1 << X_CAL)) {
		step_stop(X);
		EXTI->PR |= EXTI_PR_PIF4;
	} else if (GPIOC->IDR & (1 << Y_CAL)) {
		step_stop(y);
		EXTI->PR |= EXTI_PR_PIF5;
	}
}

void TIM2_IRQHandler(void) {
	step();
}

void step_init(void) {
	step_stop(X);
	step_stop(Y);
}

void step_stop(int axis) {
	if (axis == X) {
		stepx = OFF;
		TIM3->CCR3 = 0;
		gpio_write_reg16(&(GPIOC->ODR), X_SLEEP, 0);
	} else if (axis == Y) {
		stepy = OFF;
		TIM3->CCR4 = 0;
		gpio_write_reg16(&(GPIOC->ODR), Y_SLEEP, 0);
	}
}

void step(void) {
	if (stepx > 0) {
		stepx--;
	} else if (stepx == 0) {
		step_stop(X);
	} 
	
	if (stepy > 0) {
		stepy--;
	} else if (stepy == 0){
		step_stop(Y);
	}
	TIM2->SR &= ~(TIM_SR_UIF);
}

/*
	Rotate n steps
*/
void stepn(int axis, int n, int dir) {
	// don't step if currently stepping
	if (axis == X && stepx == OFF) {
		stepx = n-1;
		TIM3->CCR3 = DUTY_CYCLE;
		gpio_write_reg16(&(GPIOC->ODR), X_SLEEP, 1); // wake up
		gpio_write_reg16(&(GPIOC->ODR), X_DIR, dir);
	}
	else if (axis == Y && stepy == OFF) {
		stepy = n-1;
		TIM3->CCR4 = DUTY_CYCLE;
		gpio_write_reg16(&(GPIOC->ODR), Y_SLEEP, 1);
		gpio_write_reg16(&(GPIOC->ODR), Y_DIR, dir);
	}	
}

void step_mm(int axis, int mm, int dir) {
	static int x = 0;
	int adjust = 0;
	int steps = 0;
	
	// handle slight step error
	if (x == 3) {
		adjust = 1;
		x = 0;
	}
	
	steps = (mm*STEPS_PER_MM_C)-adjust;
	stepn(axis, steps, dir);
	x++;
}

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
		*reg &= ~(3 << pin);
}

/**
 *	Toggles the GPIO pin (16 bit register).
 *	Note that the 16 bit refers to the fact that the register uses
 *	only the bottom 16 bits, where each GPIO pin occupies 1 bit.
 */
void gpio_toggle_reg16(__IO uint32_t * reg, uint32_t pin) {
	*reg ^= (1 << pin);
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
