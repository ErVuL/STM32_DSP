/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pdm2pcm.h"
#include "usb_device.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>
//#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2S2_BUFLEN 16384
#define I2S3_BUFLEN (I2S2_BUFLEN/8)
#define BUFLEN    	(I2S2_BUFLEN/8) // sample
#define FREQ_SAMP	93750 // Hz
#define FIRQ31_NTAP	128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
/* USER CODE BEGIN PV */
extern volatile _Bool HOST_PORT_COM_OPEN;  	// Port COM fully connected
extern volatile _Bool CDC_RX_DATA_PENDING; 	// Data available from port COM
uint8_t I2S2_rxState;						// I2S2 rx CallBack state
uint8_t I2S2_txState;						// I2S2 tx CallBack state
uint8_t I2S3_rxState;						// I2S3 rx CallBack state
enum I2S_state {Busy, HalfCplt, Cplt};		// I2S CallBack states definition
uint16_t I2S2_rxBuf[I2S2_BUFLEN]; 			// I2S2 rxBuffer for PmodI2S2
uint16_t I2S2_txBuf[I2S2_BUFLEN]; 			// I2S2 txBuffer for PmodI2S2
uint16_t I2S3_rxBuf[I2S3_BUFLEN]; 			// I2S3 rxBuffer for PDM Mic
uint16_t *pI2S2_txBuf = I2S2_txBuf;			// |
uint16_t *pI2S2_rxBuf = I2S2_rxBuf;			// | automatic ptr for read and write functions
uint16_t *pI2S3_rxBuf = I2S3_rxBuf;			// |
enum MainTask {WAIT, PROCESS, RECORD};		// Main task definition
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_CRC_Init(void);
static void MX_I2S3_Init(void);
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
	uint8_t Task = WAIT;
	q31_t Lbuf[BUFLEN]; 					// Left channel
	q31_t Rbuf[BUFLEN]; 					// Right Channel
  // q31_t Mbuf[BUFLEN]; 					// Mono Channel
  // q31_t pState[FIRQ31_NTAP+BUFLEN-1];
  // const q31_t pCoeffs[FIRQ31_NTAP];
  // arm_fir_instance_q31 *FIR_q31;
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
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_USB_DEVICE_Init();
  MX_CRC_Init();
  MX_I2S3_Init();
  MX_PDM2PCM_Init();
  /* USER CODE BEGIN 2 */
	char cmd[APP_RX_DATA_SIZE];
	HAL_Delay(1500);
	CDC_Clear();
	_printf("\r\n               ================");
	_printf("\r\n               *** DSP V1.0 ***");
	_printf("\r\n               ================\r\n\n");
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_SET)
	{	_cprintf("/!\\ ERROR : Hardware initialization problem !\r\n");
	}else
	{	_cprintf("Hardware initialization done\r\n");
	}

	/* Start I2S communiation */
	if((HAL_I2SEx_TransmitReceive_DMA(&hi2s2, I2S2_txBuf, I2S2_rxBuf, I2S2_BUFLEN / 2) != HAL_OK)
			|| (HAL_I2S_Receive_DMA(&hi2s3, I2S3_rxBuf, I2S3_BUFLEN) != HAL_OK))
	{	_cprintf("/!\\ ERROR : Unable to launch I2S DMA transfert !\r\n");
	}else
	{	_cprintf("I2S communication established\r\n");
	}
	CDC_rxPrintf_ON();

	/* Initialize FIR  Filter */
 // arm_fir_init_q31(FIR_q31, FIRQ31_NTAP, pCoeffs, pState, BUFLEN);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

		/* Command from Port COM available */
		if (CDC_RX_DATA_PENDING)
		{
			/* Read command */
			_scanf("%s", cmd);
			CDC_RX_DATA_PENDING = FALSE;

			/* Execute command */
			if (!strcmp(cmd, "clear"))
			{	CDC_Clear();
			}
			else if((!strcmp(cmd, "q") || !strcmp(cmd, "Q")) && Task != WAIT)
			{
				Task = WAIT;
				_cprintf("Process stopped by user\r\n");
				CDC_rxPrintf_ON();
			}
			else if (!strcmp(cmd, "process") && Task != PROCESS)
			{
				Task = PROCESS;
				CDC_rxPrintf_OFF();
				_cprintf("Processing, type \"q\" to stop\r\n");
			}
			else
			{
				_cprintf("/!\\ Unknown Command !\r\n");
			}
		}

		/* Execute selected task */
		switch(Task)
		{
			case(PROCESS):
				{
					/* Read audio data */
				 // MP45DT02_monoRq31(M_Buffer); // do not work actually !
					PMODI2S2_stereoRq31(Lbuf, Rbuf);

					/* Signal Processing */
				 // arm_fir_q31(FIR_q31, L_Buf, L_Buf, BUFFER_LENGTH);

					/* Write audio data */
					PMODI2S2_stereoWq31(Lbuf, Rbuf);
					_cspin();
					break;
				}
			default:
				break;
		}

		/* Toggle Led and update chrono on port COM */
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_CRC_DR_RESET(&hcrc);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */
  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_MSB;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	pI2S2_txBuf = &I2S2_txBuf[0];
	pI2S2_rxBuf = &I2S2_rxBuf[0];
	I2S2_rxState = HalfCplt;
	I2S2_txState = HalfCplt;
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	pI2S2_txBuf = &I2S2_txBuf[I2S2_BUFLEN/2];
	pI2S2_rxBuf = &I2S2_rxBuf[I2S2_BUFLEN/2];
	I2S2_rxState = Cplt;
	I2S2_txState = Cplt;
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	pI2S3_rxBuf = &I2S3_rxBuf[0];
	I2S3_rxState = HalfCplt;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	pI2S3_rxBuf = &I2S3_rxBuf[I2S3_BUFLEN/2];
	I2S3_rxState = Cplt;
}

void MP45DT02_monoRq31(q31_t *Mbuf)
{
	while(I2S3_rxState == Busy)
	{
	}
	I2S3_rxState = Busy;
	PDM_Filter(pI2S3_rxBuf, Mbuf, &PDM1_filter_handler);
	_printf("Value = %d\r\n", pI2S3_rxBuf[12]);
	_printf("Value     = %d\r\n", pI2S3_rxBuf[11]);
}

void PMODI2S2_stereoRq31(q31_t *Lbuf, q31_t *Rbuf)
{
	while(I2S2_rxState == Busy)
	{
	}
	I2S2_rxState = Busy;
	for (uint16_t i = 0; i + 3 < I2S2_BUFLEN/2; i += 4)
	{
		Lbuf[i / 4] = (q31_t) ((pI2S2_rxBuf[i]     << 16) | pI2S2_rxBuf[i + 1]);
		Rbuf[i / 4] = (q31_t) ((pI2S2_rxBuf[i + 2] << 16) | pI2S2_rxBuf[i + 3]);
	}
}

void PMODI2S2_stereoWq31(q31_t *Lbuf, q31_t *Rbuf)
{
	while(I2S2_txState == Busy)
	{
	}
	I2S2_txState = Busy;
	for (uint16_t i = 0; i + 3 < I2S2_BUFLEN/2; i += 4)
	{
		pI2S2_txBuf[i]     = (Lbuf[i / 4] >> 16) & 0xFFFF;
		pI2S2_txBuf[i + 1] =  Lbuf[i / 4] & 0xFFFF;
		pI2S2_txBuf[i + 2] = (Rbuf[i / 4] >> 16) & 0xFFFF;
		pI2S2_txBuf[i + 3] =  Rbuf[i / 4] & 0xFFFF;
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
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	_cprintf("/!\\ FATAL ERROR !");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
