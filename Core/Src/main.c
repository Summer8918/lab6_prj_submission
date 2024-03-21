/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void initGPIOAsADCPPin(void);
void initLEDs(void);
void lab_6_1(void);
void initGPIOAsDACPPin(void);
void initUsart3(void);
void turnUint32ToBinaryStr(uint32_t x);

void initUsart3(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to PC
	// PC4 TX, PC5 RX
	// set pc4 to AF mode, 0x10
	GPIOC->MODER |= (1 << 9);
	GPIOC->MODER &= ~(1 << 8);
	// set pc5 to AF mode, 0x10
	GPIOC->MODER |= (1 << 11);
	GPIOC->MODER &= ~(1 << 10);
	
	// set PC4 AFRL to 0001: AF1
	GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFRL4_Pos);
	// set PC5 AFRL to 0001: AF1
	GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFRL5_Pos);
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	uint32_t fClk = HAL_RCC_GetHCLKFreq();
	
	// set baud rate
	uint32_t baudRate = 115200;
	uint32_t usartBRR = fClk / baudRate;
	USART3->BRR = usartBRR;

	// enable the transmitter and receiver hardware of USART3
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	
	// Enable USART peripheral.
	USART3->CR1 |= USART_CR1_UE;
}

void transmitOneChar(uint8_t ch) {
  while ((USART3->ISR & USART_ISR_TXE) == 0) {
	}
	USART3->TDR = ch;
}

void transmitCharArray (char *arr) {
  while (*arr != '\0') {
		transmitOneChar(*arr);
		arr++;
	}
}

void initLEDs(void) {
	// red LED PC6, blue LED (PC7), green LED PC9, orange LED PC8
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to PC
  // set the MODER, 01: General purpose output mode
	// init PC6 MODER
	GPIOC->MODER |= (1 << 12);
	GPIOC->MODER &= ~(1 << 13);
  // init PC7 MODER
	GPIOC->MODER |= (1 << 14);
	GPIOC->MODER &= ~(1 << 15);
	// init PC8 MODER
	GPIOC->MODER |= (1 << 16);
	GPIOC->MODER &= ~(1 << 17);
	// init PC9 MODER
	GPIOC->MODER |= (1 << 18);
	GPIOC->MODER &= ~(1 << 19);
	// Set the pins to low speed in the OSPEEDR register
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 13));
	GPIOC->OSPEEDR &= ~((1 << 14) | (1 << 15));
	GPIOC->OSPEEDR &= ~((1 << 16) | (1 << 17));
	GPIOC->OSPEEDR &= ~((1 << 18) | (1 << 19));
	
	// Set LED to no pull-up/down resistors in the PUPDR register
	// 00: No pull-up, pull-down
	GPIOC->PUPDR &= ~((1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15));
	// set PC6-9 to 1
	GPIOC->ODR |= (1 << 6);
	GPIOC->ODR |= (1 << 7);
	GPIOC->ODR |= (1 << 8);
	GPIOC->ODR |= (1 << 9);
}

void initGPIOAsADCPPin(void) {
  // PC0, ADC IN 10
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to PC
	// Configure the pin to analog mode
	GPIOC->MODER |= ((1 << 0) | (1 << 1));
	// no pull-up/down resistors, 00
	GPIOC->PUPDR &= ~((1 << 0) | (1 << 1));
	//  Enable the ADC1 in the RCC peripheral.
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	// Configure the ADC to 8-bit resolution, 10: 8 bits
	ADC1->CFGR1 |= (0x2 << ADC_CFGR1_RES_Pos);
	// continuous conversion mode
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	//  hardware triggers disabled (software trigger only)
	ADC1->CFGR1 &= (~ADC_CFGR1_EXTEN_Msk);
	// Enable channel 10,  Select/enable the input pin’s channel for ADC conversion.
	ADC1->CHSELR |= ADC_CHSELR_CHSEL10;
	
	// Perform a self-calibration
	if ((ADC1->CR & ADC_CR_ADEN) != 0) {
	  ADC1->CR &= (~ADC_CR_ADEN);
	}
	if((ADC1->CFGR1 & ADC_CFGR1_DMAEN) != 0) {
		ADC1->CFGR1 &= (~ADC_CFGR1_DMAEN);
	}
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) {
	}
  // Clear the ADRDY bit in ADC_ISR register by programming this bit to 1. 
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) {
	  ADC1->ISR |= ADC_ISR_ADRDY;
	}
	
	// enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	// Wait until ADRDY = 1 in the ADC_ISR register
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
	}
	
	ADC1->CR |= ADC_CR_ADSTART;
}

void lab_6_2(void) {
	initUsart3();
	// PA4, DAC_OUT1
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable peripheral clock to PA
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	// Configure the pin to analog mode
	GPIOA->MODER |= ((1 << 8) | (1 << 9));
	// no pull-up/down resistors, 00
	GPIOA->PUPDR &= ~((1 << 8) | (1 << 9));
	// Set the used DAC channel to software trigger mode
	DAC1->SWTRIGR |= (0x1);
	// Enable DAC channel 1
	DAC1->CR |= DAC_CR_EN1;
	uint16_t index = 0;
	// Sine Wave: 8-bit, 32 samples/cycle
  const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
      232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	while (1) {
		//transmitCharArray("write data:\n");
		//turnUint32ToBinaryStr(sine_table[index % 32]);
	  DAC1->DHR8R1 = sine_table[index % 32];
		index++;
		HAL_Delay(1);
	}
}

void turnUint32ToBinaryStr(uint32_t x){
	char str[32];
	uint32_t i = 0;
	while (i < 32) {
		str[31-i] = '0' + ((x >> i) & 0x1);
	  i++;
	}
	transmitCharArray(str);
}

void lab_6_1(void) {
  initLEDs();
	initUsart3();
	initGPIOAsADCPPin();
	int16_t minVal = 0;
	int16_t maxVal = 0;
	int16_t th1 = 0, th2 = 0, th3 = 0;
	int16_t prevData = 0;
	while (1) {
	  int16_t data = ADC1->DR;
		if (prevData != data) {
		  transmitCharArray("Data:\n");
		  turnUint32ToBinaryStr(data);
		}
		prevData = data;
		if ( data > maxVal) {
		  maxVal = data;
		}
		if (data < minVal) {
		  minVal = data;
		}
		th1 = 60;//minVal + (maxVal - minVal) / 4;
		th2 = 120; //minVal + (maxVal - minVal) / 2;
		th3 = 180;//minVal + 3 * (maxVal - minVal) / 4;
		if (data <= th1) {
		  GPIOC->ODR |= (1 << 6);
			GPIOC->ODR &= ~((1 << 7) | (1 << 8) | (1 << 9));
		} else if (data <= th2) {
		  GPIOC->ODR |= (1 << 7);
			GPIOC->ODR &= ~((1 << 6) | (1 << 8) | (1 << 9));
		} else if (data <= th3) {
		  GPIOC->ODR |= (1 << 8);
			GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 9));
		} else {
		  GPIOC->ODR |= (1 << 9);
			GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8));
		}
	}
}

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
	// lab_6_1();
	lab_6_2();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
