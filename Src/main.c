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
#include "gpio.h"
#include "stepper.h"
#include "stepper_control.h"
#include "serial.h"
#include "string.h"
#include "hall_array_library.h"


//#define CAL_PRIORITY  0
#define STEP_PRIORITY 1

char rx_char;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

// Retargets the C library printf function to the USART
PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (DEBUG) printf(&rx_char);
	if (rx_serial_command(rx_char, NULL) == FAIL) {
		SEND_CMD_P(CMD_STATUS, "%d", STATUS_FAIL);
	}
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_char, 1);   //activate UART receive interrupt every time
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*
 *	initialize user button
 */
void  button_init(void) {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}

/*
 *	initialize PWM timers for step control
 */
void timer_init(void) {
	RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // PWM
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // for interrupts, mirror PWM
	
	// for interrupt
	TIM2->PSC   = PRESCALE;
	TIM2->ARR   = AUTO_RELOAD;   
	TIM2->DIER |= TIM_DIER_UIE;
	
	// PWM waveforms for stepper driver DRV8824
	TIM3->PSC = PRESCALE;	// frequency = (8MHz/PSC/ARR) *approximatly*
	TIM3->ARR = AUTO_RELOAD;
	// duty cycle: CCR is % of ARR register (ex. ARR = 10, CCR = 1 => 10% duty cycle)
	TIM3->CCR3 = 0; // PC8
	TIM3->CCR4 = 0; // PC9
	TIM3->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos); // ch 3 capture/compare PWM mode 1
	TIM3->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos); // ch 4 capture/compare PWM mode 1
	TIM3->CCER  |= TIM_CCER_CC3E; // enable ch 3
	TIM3->CCER  |= TIM_CCER_CC4E; // enable ch 4
	
	//NVIC_SetPriority(TIM2_IRQn, STEP_PRIORITY);
	//NVIC_EnableIRQ(TIM2_IRQn);
	
	TIM2->CR1 |= TIM_CR1_CEN; // enable timers
	TIM3->CR1 |= TIM_CR1_CEN;
}

/*
 *	initialize PWM output pins
 */
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

/*
 *	initialize calibration switches and 
 *  set up EXTI interrupts
 */
void cal_switches_init(void) {
	RCC->AHBENR  |= RCC_AHBENR_GPIOCEN;
	
	gpio_output_init(GPIOC, X_RESET); // PC2 (x-axis reset)
	gpio_output_init(GPIOC, Y_RESET); // PC3 (y-axis reset)
	
	gpio_input_init(GPIOC, X_CAL);
	gpio_input_init(GPIOC, Y_CAL);
	
	// resets internal step counters in DRV8824
	gpio_write_reg16(&(GPIOC->ODR), X_RESET, 1); // nRESET
	gpio_write_reg16(&(GPIOC->ODR), Y_RESET, 1);
}

void sys_init(){
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;//enable clock for gpioc - analog in
	//PB10 -ADC PB15 are used for data select lines for MUX
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	
	//configure GPIO 10 - 15 to general purpose output mode
	gpio_output_init(GPIOB, MAGNET_PIN); // PB2
	GPIOB->MODER |= GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0;
	GPIOB->MODER |= GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0;
	GPIOB->MODER |= GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0;
	
	//enable alternate function mode for USART, with pull up resistor and open drain
	GPIOB->MODER |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_6;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0;

	//enable clock for ADC
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	//ADC1->CFGR1 &= ~ADC_CFGR1_CONT; // enable single conversion
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10; //PC0 is hooked to channel 10
	//calibrate ADC
	if((ADC1->CR & ADC_CR_ADEN) != 0){ //disable ADC if needed
		ADC1->CR |= ADC_CR_ADDIS;
	}
	while((ADC1->CR & ADC_CR_ADEN) != 0){
		/*implement a time-out here*/
	}
	ADC1->CR |= ADC_CR_ADCAL;
	while((ADC1->CR & ADC_CR_ADCAL) != 0){
		/*implement a time-out here*/
	}
	/*Enable sequence code*/
	if((ADC1->ISR & ADC_ISR_ADRDY) != 0){
		ADC1->ISR |= ADC_ISR_ADRDY;
	}
	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0){
		/*implement a time out here*/
	}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  // LEDS on PC8, PC9, PC6, PC7 DON'T USE THESE PINS FOR TIMERS
  // Use TIM2_CH2 (PA1) and TIM3_CH4 (PB1)
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  button_init();
  timer_init();
  output_init();
  cal_switches_init();
  sys_init();
  HAL_Delay(4000);
  init_board();
  
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_char, 1);
  LOG_INFO("system ready, place pieces");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	__WFI();
	  
	  get_board_state();
	  HAL_Delay(2000);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
