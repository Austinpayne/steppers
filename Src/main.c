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
#include "include/stepper_control.h"
#include "include/uart.h"
#include "string.h"

#define UART_PRIORITY 1
#define CAL_PRIORITY  0
#define STEP_PRIORITY 1
#define SERIAL_BUFF_SIZE 64
#define SERIAL_READ rx_char

unsigned char calibrating = 1;
char rx_char;
char rx_buffer[SERIAL_BUFF_SIZE];
int irx = 0;

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

void rx_serial_command() {
    if (irx < SERIAL_BUFF_SIZE) {
        char c = SERIAL_READ;
        if (c == '\r' || c == '\n') {
            rx_buffer[irx] = '\0';
            printf("%s", rx_buffer);
            memset(rx_buffer, 0, SERIAL_BUFF_SIZE);
            irx = 0;
        } else {
            rx_buffer[irx] = c;
            irx++;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_char, 1);   //activate UART receive interrupt every time
	}
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
	
	NVIC_SetPriority(TIM2_IRQn, STEP_PRIORITY);
	NVIC_EnableIRQ(TIM2_IRQn);
	
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
	gpio_output_init(GPIOC, MAGNET_PIN); // PC1
	
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

void cal_interrupt_init(void) {
	cal_switches_init();
	// enable cal switch interrupts
	// switches are active low
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PC; // multiplex PC4 to EXTI4
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PC; // multiplex PC5 to EXTI5
	EXTI->IMR |= (EXTI_IMR_IM4 | EXTI_IMR_IM5); // unmask EXTI4 & EXTI5
	EXTI->FTSR |= (EXTI_RTSR_RT4 | EXTI_RTSR_RT5); // trigger on rising edge
	
	//NVIC_EnableIRQ(EXTI4_15_IRQn); // enable interrupt in NVIC
	//NVIC_SetPriority(EXTI4_15_IRQn, CAL_PRIORITY);
}

void calibrate(void) {
	// step until hit calibration switches
		
	add_to_queue(-2000, 0);
	
	while (!(GPIOC->IDR & (1 << X_CAL))) {
		// wait until x interrupt
		// do something at timeout
    }
	stop_axis(X);
	add_to_queue(SQUARE_HALF_WIDTH, -2000);
	
	while (!(GPIOC->IDR & (1 << Y_CAL))) {
		// wait until y interrupt
    }
	stop_axis(Y);
	add_to_queue(0, SQUARE_HALF_WIDTH);
	HAL_Delay(1000);
	calibrating = 0;
}

/*
 *	initialize uart peripheral
 */
void uart_init(void) {
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	// setup uart
	GPIOA->MODER  |= (2 << GPIO_MODER_MODER9_Pos);  // alternate
	GPIOA->MODER  |= (2 << GPIO_MODER_MODER10_Pos); // alternate
	GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFRH1_Pos); // PA9,  USART1_TX
	GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFRH2_Pos); // PA10, USART1_RX
	USART1->BRR    = 833; // set baud to 9600 = 8MHz/833
	USART1->CR1   |= USART_CR1_TE; // enable TX
	USART1->CR1   |= USART_CR1_RXNEIE; // enable RXNE interrupt
	USART1->CR1   |= USART_CR1_RE; // enable RX
	
	NVIC_SetPriority(USART1_IRQn, UART_PRIORITY);
	NVIC_EnableIRQ(USART1_IRQn);
	
	USART1->CR1   |= USART_CR1_UE; // enable USART1
}

/*
 *	for user button (kill switch)
 */
void HAL_SYSTICK_Callback(void) {
    static uint32_t debouncer = 0;
	static uint32_t x_debouncer = 0;
	static uint32_t y_debouncer = 0;
    
    debouncer = (debouncer << 1);
    if(GPIOA->IDR & (1 << 0)) {
        debouncer |= 0x1;
    }

    if(debouncer == 0x7FFFFFFF) {
		if (stepping()) { // kill switch
			stop_stepping();
			empty_queue();
		} else { // send ok to photon
			tx_cmd_char(CMD_STATUS, OK);
		}
    }
	
	if (!calibrating) {
		if (axis_stepping(X)) {
			x_debouncer = (x_debouncer << 1);
			if (GPIOC->IDR & (1 << X_CAL)) {
				x_debouncer |= 0x1;
			}
			if (x_debouncer == 0x7FFFFFFF) {
				x_debouncer = 0;
				stop_stepping();
				empty_queue();
				add_to_queue(SQUARE_HALF_WIDTH, 0);
			}
		}
		
		if (axis_stepping(Y)) {
			y_debouncer = (y_debouncer << 1);
			if (GPIOC->IDR & (1 << Y_CAL)) {
				y_debouncer |= 0x1;
			}
			if (y_debouncer == 0x7FFFFFFF) {
				y_debouncer = 0;
				stop_stepping();
				empty_queue();
				add_to_queue(0, SQUARE_HALF_WIDTH);
			}
		}
	}
    
}

#if 0
/*
 *	calibration switches
 *  (broken, for some reason needs two clicks)
 */
void EXTI4_15_IRQHandler(void) {
	static uint8_t x_debouncer = 0;
	static uint8_t y_debouncer = 0;
    
    x_debouncer = (x_debouncer << 1);
    if (GPIOC->IDR & (1 << X_CAL)) {
		stop_stepping();
        //x_debouncer |= 0x1;
    }
	y_debouncer = (y_debouncer << 1);
    if (GPIOC->IDR & (1 << Y_CAL)) {
		stop_stepping();
        //y_debouncer |= 0x1;
    }
	
	if (x_debouncer == 0x7F) {
		//x_debouncer = 0;
		stop_stepping();
		empty_queue();
		//sadd_to_queue(SQUARE_HALF_WIDTH, 0);
		EXTI->PR |= (1 << X_CAL); // clear flag
	} 
	if (y_debouncer == 0x7F) {
		//y_debouncer = 0;
		stop_stepping();
		empty_queue();
		//add_to_queue(0, SQUARE_HALF_WIDTH);
		EXTI->PR |= (1 << Y_CAL);
	}
}
#endif

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
  step_control_init();
  cal_switches_init();
  calibrate();
  step_reset(); // set beginning position
  cal_interrupt_init();
  
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_char, 1);

  /*int i;
  for (i=0; i < 10; i++) {
	add_to_queue_d(50, 0, magnet_on);
	add_to_queue(0, 50);
	add_to_queue(-50, 0);
	add_to_queue(0, -50);
  }*/
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  printf("%s\n", "Hello World!");
  /* USER CODE BEGIN 3 */
  __WFI();
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
