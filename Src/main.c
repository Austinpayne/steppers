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

/* USER CODE BEGIN Includes */
#include "include/gpio.h"
#include "include/stepper.h"
#include "include/queue.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
step_queue_t steps;
char uart_rx_buffer[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);


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
	/*
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PC; // multiplex PC4 to EXTI4
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PC; // multiplex PC5 to EXTI5
	EXTI->IMR |= (EXTI_IMR_IM4 | EXTI_IMR_IM5); // unmask EXTI4 & EXTI5
	EXTI->RTSR |= (EXTI_RTSR_RT4 | EXTI_RTSR_RT5); // trigger on rising edge
	NVIC_EnableIRQ(EXTI4_15_IRQn); // enable interrupt in NVIC
	NVIC_SetPriority(EXTI4_15_IRQn, 1);
	*/
	
	// TODO: move axis until they hit cal switches then clear counters
	
	gpio_write_reg16(&(GPIOC->ODR), X_RESET, 1); // nRESET
	gpio_write_reg16(&(GPIOC->ODR), Y_RESET, 1);
	
}

void uart_init(void) {
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// setup uart
	GPIOA->MODER  |= (2 << GPIO_MODER_MODER9_Pos);  // alternate
	GPIOA->MODER  |= (2 << GPIO_MODER_MODER10_Pos); // alternate
	GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFRH1_Pos); // PA9,  USART1_TX
	GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFRH2_Pos); // PA10, USART1_RX
	USART1->BRR    = 69; // set baud to 115200 = 8MHz/69
	USART1->CR1   |= USART_CR1_TE; // enable TX
	USART1->CR1   |= USART_CR1_RXNEIE; // enable RXNE interrupt
	USART1->CR1   |= USART_CR1_RE; // enable RX
	
	NVIC_SetPriority(USART1_IRQn, 0);
	NVIC_EnableIRQ(USART1_IRQn);
	
	USART1->CR1   |= USART_CR1_UE; // enable USART1
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
	
	init(&steps);
	add(&steps, 3, -3);
	add(&steps, -3, 3);
	add(&steps, 10, -4);
	add(&steps, -10, 6 );
	
	while (1)
	{
		__WFI();  
	}
}

void USART1_IRQHandler(void) {
	static int i = 0;
	char temp = USART1->RDR;

	if (temp == '\r' || temp == '\n') {
		uart_rx_buffer[i++] = '\0'; // terminate
		// now process
		char *cmd = strtok(uart_rx_buffer, " ");
	} else {
		uart_rx_buffer[i++] = temp;
	}
}

void HAL_SYSTICK_Callback(void) {
    static uint32_t debouncer = 0;
    
    debouncer = (debouncer << 1);
    if(GPIOA->IDR & (1 << 0)) {
        debouncer |= 0x1;
    }

    if(debouncer == 0x7FFFFFFF) {
		step_squares(X, 2); // step 30mm, about one rotation using 1/16 microstep
		step_squares(Y, 4);
    }
    
}

void EXTI4_15_IRQHandler(void) {
	if (GPIOC->IDR & (1 << X_CAL)) {
		step_stop(X);
		EXTI->PR |= EXTI_PR_PIF4;
	} else if (GPIOC->IDR & (1 << Y_CAL)) {
		step_stop(Y);
		EXTI->PR |= EXTI_PR_PIF5;
	}
}

void TIM2_IRQHandler(void) {
	// if not stepping, get next step from queue
	if (!empty(&steps) && get_steps(X) == OFF && get_steps(Y) == OFF) {
		steps_t next = rm(&steps);
		step_squares(X, next.x_steps);
		step_squares(Y, next.y_steps);
	} else {
		step();
	}
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
