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
I2S_HandleTypeDef hi2s2;

SD_HandleTypeDef hsd;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

FATFS fs;


volatile bool end_of_file_reached = false;
volatile bool read_next_chunk = false;
volatile uint16_t* signal_play_buff = NULL;
volatile uint16_t* signal_read_buff = NULL;
volatile uint16_t signal_buff1[4096];
volatile uint16_t signal_buff2[4096];

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

    uint32_t fileSize = 8 + (header[4] | (header[5] << 8) | (header[6] << 16) | (header[7] << 24));
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

    return 0;
}

int playWavFile_Me(const char* fname) {

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

	uint32_t fileSize = 8 + (header[4] | (header[5] << 8) | (header[6] << 16) | (header[7] << 24));
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

	volatile uint16_t signal_buff[16384];
	 HAL_StatusTypeDef hal_res;
	 int nsamples = sizeof(signal_buff) / sizeof(signal_buff[0]);

	 uint32_t filePointerOffsetCount = 0;

	while(1) {
		res = f_lseek(&file, ((sizeof(signal_buff) / 16) * filePointerOffsetCount) + 45);
		res = f_read(&file, (uint8_t*)signal_buff, sizeof(signal_buff) / 16, &bytesRead);
		if(res != FR_OK) {
			//UART_Printf("f_read() failed, res = %d\r\n", res);
			f_close(&file);
			return EXIT_FAILURE;
		}


		hal_res = HAL_I2S_Transmit(&hi2s2, (uint16_t*)signal_buff, nsamples, 1000);
		if(hal_res != HAL_OK) {
			//UART_Printf("I2S - HAL_I2S_Transmit failed, hal_res = %d!\r\n", hal_res);
			f_close(&file);
			return EXIT_FAILURE;
		}

		filePointerOffsetCount++;

		//HAL_Delay(1000);

	}


}

int playWavFile_Test(const char* fname) {

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

    uint32_t fileSize = 8 + (header[4] | (header[5] << 8) | (header[6] << 16) | (header[7] << 24));
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  FRESULT res = f_mount(&fs, "XMAS-23", 1);
   if(res != FR_OK) {
	   return EXIT_FAILURE;
   }

  while (1)
  {


	  playWavFile("canS.wav");
	  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
	   {
	     Error_Handler();
	   }

//	  playWavFile("1k.wav");
//	  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
//	   {
//	     Error_Handler();
//	   }

//	  res = f_mount(0, "XMAS-23", 0);
//	  if(res != FR_OK) {
//		   return EXIT_FAILURE;
//	   }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2S2;
  PeriphClkInit.I2s2ClockSelection = RCC_I2S2CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_GPIO_WritePin(I2S_AMP_SD_GPIO_Port, I2S_AMP_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STAT_LED_GPIO_Port, STAT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : I2S_AMP_SD_Pin */
  GPIO_InitStruct.Pin = I2S_AMP_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2S_AMP_SD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STAT_LED_Pin */
  GPIO_InitStruct.Pin = STAT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STAT_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_CARD_DETECT_Pin */
  GPIO_InitStruct.Pin = SDIO_CARD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDIO_CARD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_CD_FAKE_Pin */
  GPIO_InitStruct.Pin = SDIO_CD_FAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDIO_CD_FAKE_GPIO_Port, &GPIO_InitStruct);

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
