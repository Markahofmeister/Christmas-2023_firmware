/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include "wav.h"
#include <stdarg.h>
#include <math.h>

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

I2S_HandleTypeDef hi2s2;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

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

// Amplifier Gain Control Pins
uint16_t gain_3DB_N = GPIO_PIN_0;			// Port C
uint16_t gain_6DB_N = GPIO_PIN_1;			// Port C
uint16_t gain_12DB = GPIO_PIN_2;			// Port C
uint16_t gain_15DB = GPIO_PIN_3;			// Port C

// For setting GPIO high or low in a more pretty fashion
GPIO_PinState GPIOPinSet[2] = {GPIO_PIN_RESET, GPIO_PIN_SET};

// Play index keeps track of which wav file should be
// played on the next playWavFile() call
static uint8_t playIndex = 0;

/*
 *  Volume keeps track of which amplifier gain should be selected
 *  Will take on values in the range 0-4
 *  0 = 3dB
 *  1 = 6dB
 *  2 = 9dB
 *  3 = 12dB
 *  4 = 15dB
 */
static uint8_t volume = 2;

/*
 * Array of all possible LED volume bar values that
 * might be shifted into the shift register.
 * Ordered highest volume display to lowest volume display
 */
static uint8_t shiftBytes[8] = {0b10000000, 0b01000000, 0b00100000, 0b00010000,
								0b00001000, 0b00000100, 0b00000010, 0b00000001};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_I2S2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

FATFS fs;						// Object to set up FATFS on SD card

/*
 * flags and buffers for data loading and transmission
 */
volatile bool end_of_file_reached = false;
volatile bool read_next_chunk = false;
volatile uint16_t* signal_play_buff = NULL;
volatile uint16_t* signal_read_buff = NULL;
volatile uint16_t signal_buff1[4096];
volatile uint16_t signal_buff2[4096];

/*
 * All 11 strings of file names
 * fileNames[0] = short, silent wav file that plays when idling
 * fileNames[1-10] = movie soundbites, ordered based on button order
 */

char *fileNames[] = {"blank.wav", "yard.wav", "shit.wav", "gift.wav", "nut.wav", "grace.wav",
					"dump.wav", "treeBig.wav", "kma.wav", "wint.wav", "rant.wav"};

/*
 * Callback function for I2S TX completion
 * Enters upon I2S global interrupt
 * Transmits another chunk of data
 */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if(end_of_file_reached)
        return;

    volatile uint16_t* temp = signal_play_buff;
    signal_play_buff = signal_read_buff;
    signal_read_buff = temp;

    int nsamples = sizeof(signal_buff1) / sizeof(signal_buff1[0]);
    HAL_I2S_Transmit_IT(&hi2s2, (uint16_t*)signal_play_buff, nsamples);
    read_next_chunk = true;
}

/*
 * - Opens file with file name of passed argument using f_open()
 * - Checks WAV file header format and data structuring,
 *   returns EXIT_FAILURE if a file cannot be played.
 * - Transmits data using I2S TX IT function,
 *   which will enter the TX Cplt callback function to transmit another chunk of data
 * - Enters a loop to move the file pointer when I2S is available to TX more data
 */
int playWavFile(const char* fname) {

    FIL file;
    FRESULT res = f_open(&file, fname, FA_READ);
    if(res != FR_OK) {
        return EXIT_FAILURE;
    }


    unsigned int bytesRead;
    uint8_t header[44];
    res = f_read(&file, header, sizeof(header), &bytesRead);
    if(res != FR_OK) {
        f_close(&file);
        return EXIT_FAILURE;
    }

    if(memcmp((const char*)header, "RIFF", 4) != 0) {
        f_close(&file);
        return EXIT_FAILURE;
    }

    if(memcmp((const char*)header + 8, "WAVEfmt ", 8) != 0) {
        f_close(&file);
        return EXIT_FAILURE;
    }

    if(memcmp((const char*)header + 36, "data", 4) != 0) {
        f_close(&file);
        return EXIT_FAILURE;
    }

//    uint32_t fileSize = 8 + (header[4] | (header[5] << 8) | (header[6] << 16) | (header[7] << 24));
    uint32_t headerSizeLeft = header[16] | (header[17] << 8) | (header[18] << 16) | (header[19] << 24);
    uint16_t compression = header[20] | (header[21] << 8);
    uint16_t channelsNum = header[22] | (header[23] << 8);
    uint32_t sampleRate = header[24] | (header[25] << 8) | (header[26] << 16) | (header[27] << 24);
    uint32_t bytesPerSecond = header[28] | (header[29] << 8) | (header[30] << 16) | (header[31] << 24);
    uint16_t bytesPerSample = header[32] | (header[33] << 8);
    uint16_t bitsPerSamplePerChannel = header[34] | (header[35] << 8);
    uint32_t dataSize = header[40] | (header[41] << 8) | (header[42] << 16) | (header[43] << 24);

//    UART_Printf(
//        "--- WAV header ---\r\n"
//        "File size: %lu\r\n"
//        "Header size left: %lu\r\n"
//        "Compression (1 = no compression): %d\r\n"
//        "Channels num: %d\r\n"
//        "Sample rate: %ld\r\n"
//        "Bytes per second: %ld\r\n"
//        "Bytes per sample: %d\r\n"
//        "Bits per sample per channel: %d\r\n"
//        "Data size: %ld\r\n"
//        "------------------\r\n",
//        fileSize, headerSizeLeft, compression, channelsNum, sampleRate, bytesPerSecond, bytesPerSample,
//        bitsPerSamplePerChannel, dataSize);

    if(headerSizeLeft != 16) {
        //UART_Printf("Wrong `headerSizeLeft` value, 16 expected\r\n");
        f_close(&file);
        return EXIT_FAILURE;
    }

    if(compression != 1) {
        //UART_Printf("Wrong `compression` value, 1 expected\r\n");
        f_close(&file);
        return EXIT_FAILURE;
    }

    if(channelsNum != 2) {
        //UART_Printf("Wrong `channelsNum` value, 2 expected\r\n");
        f_close(&file);
        return EXIT_FAILURE;
    }

    if((sampleRate != 44100) || (bytesPerSample != 4) || (bitsPerSamplePerChannel != 16) || (bytesPerSecond != 44100*2*2)
       || (dataSize < sizeof(signal_buff1) + sizeof(signal_buff2))) {
        //UART_Printf("Wrong file format, 16 bit file with sample rate 44100 expected\r\n");
        f_close(&file);
        return EXIT_FAILURE;
    }

    res = f_read(&file, (uint8_t*)signal_buff1, sizeof(signal_buff1), &bytesRead);
    if(res != FR_OK) {
        //UART_Printf("f_read() failed, res = %d\r\n", res);
        f_close(&file);
        return EXIT_FAILURE;
    }

    res = f_read(&file, (uint8_t*)signal_buff2, sizeof(signal_buff2), &bytesRead);
    if(res != FR_OK) {
        //UART_Printf("f_read() failed, res = %d\r\n", res);
        f_close(&file);
        return EXIT_FAILURE;
    }

    read_next_chunk = true;
    end_of_file_reached = false;
    signal_play_buff = signal_buff1;
    signal_read_buff = signal_buff2;

    HAL_StatusTypeDef hal_res;
    int nsamples = sizeof(signal_buff1) / sizeof(signal_buff1[0]);
    hal_res = HAL_I2S_Transmit_IT(&hi2s2, (uint16_t*)signal_buff1, nsamples);
    // hal_res = HAL_I2S_Transmit(&hi2s2, (uint16_t*)signal_buff1, nsamples, 2000);
    if(hal_res != HAL_OK) {
        //UART_Printf("I2S - HAL_I2S_Transmit failed, hal_res = %d!\r\n", hal_res);
        f_close(&file);
        return EXIT_FAILURE;
    }

    while(dataSize >= sizeof(signal_buff1)) {
        if(!read_next_chunk) {
            continue;
        }

        read_next_chunk = false;

        res = f_read(&file, (uint8_t*)signal_read_buff, sizeof(signal_buff1), &bytesRead);
        if(res != FR_OK) {
            //UART_Printf("f_read() failed, res = %d\r\n", res);
            f_close(&file);
            return EXIT_FAILURE;
        }

        dataSize -= sizeof(signal_buff1);
    }

    end_of_file_reached = true;

    res = f_close(&file);
    if(res != FR_OK) {
        //UART_Printf("f_close() failed, res = %d\r\n", res);
        return EXIT_FAILURE;
    }

    playIndex = 0;

    return 0;
}


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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_I2S2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // Calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc1);

  // Start timer to check ADC on interrupts
  HAL_TIM_Base_Start_IT(&htim4);

  // Clear any existing shift register data
  	HAL_GPIO_WritePin(GPIOB, shiftMCLR, GPIOPinSet[0]);
  	HAL_GPIO_WritePin(GPIOB, shiftMCLR, GPIOPinSet[1]);

  	// Store cleared data and Enable output
  	HAL_GPIO_WritePin(GPIOB, shiftStoreClock, GPIOPinSet[1]);
  	HAL_GPIO_WritePin(GPIOB, shiftStoreClock, GPIOPinSet[0]);
  	HAL_GPIO_WritePin(GPIOB, shiftOutputEnable, GPIOPinSet[0]);

  	// Initialize FAT file system within SD card
    FRESULT res = f_mount(&fs, "XMAS-23", 1);
	 if(res != FR_OK) {
	   return EXIT_FAILURE;
	 }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*
	   * Always play WavFile, which is usually blank.wav
	   * playIndex is changed by GPIO EXTI handler
	   */
	  playWavFile(fileNames[playIndex]);
	  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
		   Error_Handler();

	  /*
	   * Some GPIOs in Mk. I are incompatible with EXTI,
	   * so they require loop checking
	   */
	  if(HAL_GPIO_ReadPin(GPIOB, buttonIn_6) == GPIOPinSet[0]) {
		  playIndex = 6;
	  }
	  if(HAL_GPIO_ReadPin(GPIOB, buttonIn_7) == GPIOPinSet[0]) {
		  playIndex = 7;
	  }
	  if(HAL_GPIO_ReadPin(GPIOB, buttonIn_9) == GPIOPinSet[0]) {
		  playIndex = 9;
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_I2S2;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.I2s2ClockSelection = RCC_I2S2CLKSOURCE_SYSCLK;
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9601-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GAIN_3DB_NOT_Pin|GAIN_6DB_NOT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GAIN_12DB_Pin|GAIN_15DB_Pin|STAT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2S_AMP_SD_GPIO_Port, I2S_AMP_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHIFT_OE_Pin|SHIFT_DATA_OUT_Pin|SHIFT_DATA_CLK_Pin|SHIFT_STORE_CLK_Pin
                          |SHIFT_MCLR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GAIN_3DB_NOT_Pin GAIN_6DB_NOT_Pin GAIN_12DB_Pin GAIN_15DB_Pin
                           STAT_LED_Pin */
  GPIO_InitStruct.Pin = GAIN_3DB_NOT_Pin|GAIN_6DB_NOT_Pin|GAIN_12DB_Pin|GAIN_15DB_Pin
                          |STAT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S_AMP_SD_Pin */
  GPIO_InitStruct.Pin = I2S_AMP_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2S_AMP_SD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_1_Pin BUTTON_3_Pin BUTTON_4_Pin BUTTON_5_Pin */
  GPIO_InitStruct.Pin = BUTTON_1_Pin|BUTTON_3_Pin|BUTTON_4_Pin|BUTTON_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_2_Pin */
  GPIO_InitStruct.Pin = BUTTON_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHIFT_OE_Pin SHIFT_DATA_OUT_Pin SHIFT_DATA_CLK_Pin SHIFT_STORE_CLK_Pin
                           SHIFT_MCLR_Pin */
  GPIO_InitStruct.Pin = SHIFT_OE_Pin|SHIFT_DATA_OUT_Pin|SHIFT_DATA_CLK_Pin|SHIFT_STORE_CLK_Pin
                          |SHIFT_MCLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_CARD_DETECT_Pin */
  GPIO_InitStruct.Pin = SDIO_CARD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDIO_CARD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_10_Pin BUTTON_8_Pin */
  GPIO_InitStruct.Pin = BUTTON_10_Pin|BUTTON_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_9_IN_Pin BUTTON_7_IN_Pin BUTTON_6_IN_Pin */
  GPIO_InitStruct.Pin = BUTTON_9_IN_Pin|BUTTON_7_IN_Pin|BUTTON_6_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if(GPIO_Pin == buttonIn_1)
		playIndex = 1;
	else if(GPIO_Pin == buttonIn_2)
		playIndex = 2;
	else if(GPIO_Pin == buttonIn_3)
		playIndex = 3;
	else if(GPIO_Pin == buttonIn_4)
		playIndex = 4;
	else if(GPIO_Pin == buttonIn_5)
		playIndex = 5;
	else if(GPIO_Pin == buttonIn_6)
		playIndex = 6;
	else if(GPIO_Pin == buttonIn_7)
		playIndex = 7;
	else if(GPIO_Pin == buttonIn_8)
		playIndex = 8;
	else if(GPIO_Pin == buttonIn_9)
		playIndex = 9;
	else if(GPIO_Pin == buttonIn_10)
		playIndex = 10;

}

/*
 * Necessary to reset all hardware-controlled gain FETs
 *
 * Avoids shorting power to ground through accidental
 * switching sequences
 */
void resetGains() {
	 HAL_GPIO_WritePin(GPIOC, gain_3DB_N, GPIOPinSet[1]);
	 HAL_GPIO_WritePin(GPIOC, gain_6DB_N, GPIOPinSet[1]);
	 HAL_GPIO_WritePin(GPIOC, gain_12DB, GPIOPinSet[0]);
	 HAL_GPIO_WritePin(GPIOC, gain_15DB, GPIOPinSet[0]);
}

/*
 * Shifts in passed byte of data to shift register and
 * enables parallel output
 */
void shiftNewVol(uint8_t shiftByte) {

		for(int j = 0; j < 8; j++) {

			// Write data pin with LSB of data
			HAL_GPIO_WritePin(GPIOB, shiftData, GPIOPinSet[shiftByte & 1]);

			// Toggle clock GPIO to shift bit into register
			HAL_GPIO_WritePin(GPIOB, shiftDataClock, GPIOPinSet[1]);
			HAL_GPIO_WritePin(GPIOB, shiftDataClock, GPIOPinSet[0]);

			// Once data pin has been written and shifted out, shift data right by one bit.
			shiftByte >>= 1;

		}

		// Once all data has been shifted out, toggle store clock register to display data.

		HAL_GPIO_WritePin(GPIOB, shiftStoreClock, GPIOPinSet[1]);
		HAL_GPIO_WritePin(GPIOB, shiftStoreClock, GPIOPinSet[0]);

		return;
}

/*
 * Callback: timer has rolled over and completed one period
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback
  if (htim == &htim4)
  {
	  // Start ADC Conversion
	  HAL_ADC_Start(&hadc1);

	 // Poll ADC1 Peripheral & TimeOut = 1mSec
	  HAL_ADC_PollForConversion(&hadc1, 1);

	  // Read The ADC Conversion Result & Map It To Shift register
	  // Resolution = 12 bit, 2^12 = 4096
	  uint16_t ADC_Return = HAL_ADC_GetValue(&hadc1);

	  /*
	   * Shift in new volume bar LED value
	   */
	  uint8_t shiftByteCurr = ADC_Return / (4096 / 8);
	  shiftNewVol(shiftBytes[shiftByteCurr]);

	  // Map ADC reading to volume variable
	  volume = ADC_Return / (4096 / 5);

	  switch(volume) {
	  	  case 0:			// Set gain = 3dB
	  		  resetGains();
	  		  HAL_GPIO_WritePin(GPIOC, gain_3DB_N, GPIOPinSet[0]);
	  		  break;
	  	  case 1:			// Set gain = 6dB
	  		  resetGains();
	  		  HAL_GPIO_WritePin(GPIOC, gain_6DB_N, GPIOPinSet[0]);
	  		  break;
	  	  case 2:			// Set gain = 9dB
	  		  resetGains();

	  		  break;
	  	  case 3:			// Set gain = 12dB
	  		  resetGains();
	  		  HAL_GPIO_WritePin(GPIOC, gain_12DB, GPIOPinSet[1]);
	  		  break;
	  	  case 4:			// Set gain = 15dB
	  		  resetGains();
	  		  HAL_GPIO_WritePin(GPIOC, gain_15DB, GPIOPinSet[1]);
	  		  break;
	  	  default:

	  		  break;
	  }

  }
}



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
