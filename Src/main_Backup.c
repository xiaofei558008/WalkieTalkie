/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "SPIRIT1.h"
#include "sta350bw.h"

#include "XiFi.h"
#include "matrix keyboard.h"
#include "test.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SAI_HandleTypeDef hsai_BlockA2;
DMA_HandleTypeDef hdma_sai2_a;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SAI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t  Test0 = 0x00,
         Test1 = 0x01;

uint16_t Test2 = 0x0203,
         Test3 = 0x0405,
         Test4 = 0x0607;

uint32_t Test5 = 0x08090a0b,
         Test6 = 0x0c0d0e0f;

SpiritStatus Temp_S;
uint8_t Version[25];

uint8_t STA350[0x60];

//uint16_t Audio_Temp[16] = {
//32768,45307,55938,63041,65535,63041,55938,45307,
//32768,20229,9598, 2495, 1,    2495, 9598, 20229};

uint16_t Audio_Temp[16] = {
5555,333,6666,111,8888,10000,3557,5556,
15555,2333,36666,4111,58888,10000,43557,25556,
};

uint16_t i = 0;
uint8_t Temp =0x55;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SAI2_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
  //Enable SPI1.
  __HAL_SPI_ENABLE(&hspi1);

  //Enable SAI.
  ////__HAL_SAI_ENABLE(&hsai_BlockA2);

  //Enable Timer7.
  HAL_TIM_Base_Start_IT(&htim7);

  /* ADC DMA Start */
  //HAL_ADC_Start_DMA(&hadc, (uint32_t*)&(SPI2->DR), 1);
  //HAL_SAI_Transmit(&hsai_BlockA2, (uint8_t*)Audio_Temp, 2,10);

  /* Spirit1 Basic Mode Init */
  ////SPIRIT1_Basic_Init(&Spirit1,&Com);
  ////Test_Init();

  /* Matrix Keyboard Init */
  ////matrix_keyport_init(&matrix_key_data);

  /* STA350BW Init */
  STA350BW_Init(&STA350BW_I2C_Drv,  //SOUNDTERMINAL_Object_t* pObj,
                0x22,               //uint16_t                volume,
                32000,              //uint32_t                samplingFreq,
                NULL                //void*                   p
               );

#if 0
  //Clock Enable.
  RCC->APB2ENR |= (1 << 22);     //SAI2.
  RCC->AHB2ENR |= 0x00000006;    //GPIOB / GPIOC.

  //GPIOB
  GPIOB->AFR[1] &= 0x0f00ffff;
  GPIOB->AFR[1] |= 0xd0dd0000;
  GPIOB->MODER  &= 0x30ffffff;
  GPIOB->MODER  |= 0x8a000000;

  //GPIOC
  GPIOB->AFR[0] &= 0xf0ffffff;
  GPIOB->AFR[0] |= 0x0d000000;
  GPIOB->MODER  &= 0xffffcfff;
  GPIOB->MODER  |= 0x00002000;

  //SAI2.
  //SAI2->GCR           = 0x00000020;
  //SAI2_Block_A->CR1   = 0x00820280;
  //SAI2_Block_A->CR2   = 0x00000001;
  //
  ////SAI2_Block_A->FRCR  = 0x00050020;
  //SAI2_Block_A->FRCR  = 0x0005001f;
  //
  //SAI2_Block_A->SLOTR = 0x00010200;
  ////SAI2_Block_A->IMR   = 0x00000000;
  ////
  //////SAI2_Block_A->SR    = ;
  ////SAI2_Block_A->CLRFR = 0xffffffff;
  //////SAI2_Block_A->DR    = ;
  //SAI2_Block_A->CR1  |= 0x00010000;

#if 0
  SAI2->GCR           = 0x00000000;
  SAI2_Block_A->CR1   = 0x00120280;
  SAI2_Block_A->CR2   = 0x00000000;
  SAI2_Block_A->FRCR  = 0x00050f1f;
  SAI2_Block_A->SLOTR = 0xffff0100;
  SAI2_Block_A->CR1  |= 0x00010000;
#endif

  ////SAI1.
  //SAI1->GCR           = 0x00000000;
  //SAI1_Block_B->CR1   = 0x00120280;
  //SAI1_Block_B->CR2   = 0x00000000;
  //SAI1_Block_B->FRCR  = 0x00050f1f;
  //SAI1_Block_B->SLOTR = 0xffff0100;
  //SAI1_Block_B->CR1  |= 0x00010000;

#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //if(((uint8_t)(SAI2_Block_A->SR >> 16)) != 5)
    {
      HAL_SAI_Transmit(&hsai_BlockA2, (uint8_t*)Audio_Temp, 16, 1);

      //SAI2_Block_A->DR = ~SAI2_Block_A->DR;
    }
   // else
    {
      //SAI2_Block_A->CR1 |= 0x00010000;
    }

    ////matrix_key_scan(&matrix_key_data);

    //Com_Main_Loop(&Com);

    //RX
    ////Test_Receive_Buffer();

    //STA350BW_I2C_Read(STA350BW_ADDRESS_1, 0x00, STA350);
    //STA350BW_Read_All_Reg(STA350, 0x56);

    //if(i<65535)
    //{
    //  SAI2_Block_A->DR = (i++);
    //}
    //else
    //{
    //  i = 0;
    //}

    //Com_Send_Frame(&Com);

    //Com_Receive_Frame_IRQ(&Com);

    //Receive Simulation.
    //Com_Receive_Simulation_Test(&Com);

    //Update Receive Frame.
    //Com_Receive_Frame_IRQ(&Com);

    //Receive Manage.
    //Com_Receive_Management(&Com);

    //Com_Read_CallBack(&Com);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_SAI2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI2;
  PeriphClkInit.PLLSAI2.PLLSAI2Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI2.PLLSAI2M = 2;
  PeriphClkInit.PLLSAI2.PLLSAI2N = 35;
  PeriphClkInit.PLLSAI2.PLLSAI2P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI2.PLLSAI2R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_SAI2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_PWR_CLK_ENABLE();

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SAI2 init function */
static void MX_SAI2_Init(void)
{

  hsai_BlockA2.Instance = SAI2_Block_A;
  hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_32K;
  hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  if (HAL_SAI_InitProtocol(&hsai_BlockA2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9722;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 79;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 Spirit1_GPIO0_Pin Spirit1_GPIO3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|Spirit1_GPIO0_Pin|Spirit1_GPIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STA350BW_PWRDN_Pin PA8 PA10 */
  GPIO_InitStruct.Pin = STA350BW_PWRDN_Pin|GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Spirit1_GPIO2_Pin */
  GPIO_InitStruct.Pin = Spirit1_GPIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Spirit1_GPIO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Spirit1_GPIO1_Pin */
  GPIO_InitStruct.Pin = Spirit1_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Spirit1_GPIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Spirit1_LED_Pin Spirit1_CSN_Pin */
  GPIO_InitStruct.Pin = Spirit1_LED_Pin|Spirit1_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STA350BW_PWRDN_Pin|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Spirit1_LED_Pin|Spirit1_CSN_Pin, GPIO_PIN_RESET);

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
