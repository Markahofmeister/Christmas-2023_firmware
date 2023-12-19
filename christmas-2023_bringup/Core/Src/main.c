/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body

	Code Flag Structure:
	1 - Blink Debug LED
	2 - Test shift register functionality
	3 - Test ADC functionality
	4 - Light up debug LED on button input interrupts
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
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/*
 *******************************************************************************
 * Change flag to enter sequential stages of bringup *
 *******************************************************************************
 */
const uint8_t bringupStage = 1;

// Debug LED on PCB
const uint16_t debugLED = GPIO_PIN_5;		// Port C

/*
 * Shift register Pins
 */
uint16_t shiftData = GPIO_PIN_1;			// Port B
uint16_t shiftDataClock = GPIO_PIN_2; 		// Port B
uint16_t shiftStoreClock = GPIO_PIN_10;		// Port B
uint16_t shiftOutputEnable = GPIO_PIN_0;	// Port B
uint16_t shiftMCLR = GPIO_PIN_11;			// Port B

// ADC Pin
uint16_t ADC_IN = GPIO_PIN_1; 				// Port A

// Button Input Pins
uint16_t buttonIn_1 = GPIO_PIN_7;			// Port A
uint16_t buttonIn_2 = GPIO_PIN_4;			// Port C
uint16_t buttonIn_3 = GPIO_PIN_8;			// Port	A
uint16_t buttonIn_4 = GPIO_PIN_9;			// Port A
uint16_t buttonIn_5 = GPIO_PIN_10;			// Port A
uint16_t buttonIn_6 = GPIO_PIN_9;			// Port B
uint16_t buttonIn_7 = GPIO_PIN_8;			// Port B
uint16_t buttonIn_8 = GPIO_PIN_5;			// Port B
uint16_t buttonIn_9 = GPIO_PIN_4;			// Port B
uint16_t buttonIn_10 = GPIO_PIN_3;			// Port B

// For setting GPIO high or low in a more pretty fashion
GPIO_PinState GPIOPinSet[2] = {GPIO_PIN_RESET, GPIO_PIN_SET};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  MX_GPIO_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(bringupStage) {
		  case 1:		// Case 1 = simple debug LED blink

			  // Debug LED connected to pin C5
			  HAL_GPIO_TogglePin(GPIOC, debugLED);
			  HAL_Delay(500);

		    break;

		  case 2:		// Case 2 =  Test shift register functionality

			    // Clear any existing shift register data
				HAL_GPIO_WritePin(GPIOB, shiftMCLR, GPIOPinSet[0]);
				HAL_GPIO_WritePin(GPIOB, shiftMCLR, GPIOPinSet[1]);

				// Store cleared data and Enable output
				HAL_GPIO_WritePin(GPIOB, shiftStoreClock, GPIOPinSet[1]);
				HAL_GPIO_WritePin(GPIOB, shiftStoreClock, GPIOPinSet[0]);
				HAL_GPIO_WritePin(GPIOB, shiftOutputEnable, GPIOPinSet[0]);

				HAL_Delay(10);

				// Write data pin with 1 bit of data
				HAL_GPIO_WritePin(GPIOB, shiftData, GPIOPinSet[1]);

				// Shift high bit into pos 1
				HAL_GPIO_WritePin(GPIOB, shiftDataClock, GPIOPinSet[1]);
				HAL_GPIO_WritePin(GPIOB, shiftDataClock, GPIOPinSet[0]);


				// Display Output
				HAL_GPIO_WritePin(GPIOB, shiftStoreClock, GPIOPinSet[1]);
				HAL_GPIO_WritePin(GPIOB, shiftStoreClock, GPIOPinSet[0]);

				// Change Data pin back to low bit
				HAL_GPIO_WritePin(GPIOB, shiftData, GPIOPinSet[0]);


				for(int i = 0; i < 7; i++) {

					HAL_Delay(1000);
					// Shift low bit
					HAL_GPIO_WritePin(GPIOB, shiftDataClock, GPIOPinSet[1]);
					HAL_GPIO_WritePin(GPIOB, shiftDataClock, GPIOPinSet[0]);

					// Display Output
					HAL_GPIO_WritePin(GPIOB, shiftStoreClock, GPIOPinSet[1]);
					HAL_GPIO_WritePin(GPIOB, shiftStoreClock, GPIOPinSet[0]);
				}


		    break;

		  case 3:		// Case 3 - Test ADC functionality

			  // If ADC > 0.5 * VDD, turn on LED.

			  // If ADC < 0.5 * VDD, turn off LED.

		    break;

		  case 4: 		// Case 4 - Light up debug LED on button input interrupts

		    break;

		  default:

		   break;
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STAT_LED_GPIO_Port, STAT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHIFT_OE_Pin|SHIFT_DATA_Pin|SHIFT_DATA_CLK_Pin|SHIFT_STORE_CLK_Pin
                          |SHIFT_MCLR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STAT_LED_Pin */
  GPIO_InitStruct.Pin = STAT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STAT_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHIFT_OE_Pin SHIFT_DATA_Pin SHIFT_DATA_CLK_Pin SHIFT_STORE_CLK_Pin
                           SHIFT_MCLR_Pin */
  GPIO_InitStruct.Pin = SHIFT_OE_Pin|SHIFT_DATA_Pin|SHIFT_DATA_CLK_Pin|SHIFT_STORE_CLK_Pin
                          |SHIFT_MCLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
