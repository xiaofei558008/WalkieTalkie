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
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>

#include "SPIRIT1.h"
#include "sta350bw.h"
#include "XiFi.h"
#include "matrix keyboard.h"
#include "Audio.h"
#include "test.h"

#include "opus.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel6;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel7;
DMA_HandleTypeDef hdma_dfsdm1_flt0;
DMA_HandleTypeDef hdma_dfsdm1_flt1;

I2C_HandleTypeDef hi2c1;

SAI_HandleTypeDef hsai_BlockA2;
DMA_HandleTypeDef hdma_sai2_a;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

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
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_DFSDM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
OpusEncoder* Enc = NULL;
OpusDecoder* Dec = NULL;

uint8_t      Flag_DFSDM_Int[4];

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  static uint16_t Count;

  int16_t  Random_Data[Audio_Buffer_In_Size],
           Encode_Temp[Audio_Buffer_In_Size];

  //Communication Init.
  Com_Init(&Com);

  /* Opus Init.
  */
  int application  = OPUS_APPLICATION_AUDIO;
  int err;
  int channels = 1;

  opus_int32 sampling_rate = 48000,
                  bit_rate = 10000;

  //Encoder Create.
  Enc = opus_encoder_create(sampling_rate, channels, application, &err);

  if(err != OPUS_OK)
  {
    while(1);
  }

  /*Signal being encoded is voice*/
  err = opus_encoder_ctl(Enc, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));

  if (err != OPUS_OK)
  {
    while(1);
  }

  /*Bitrate set*/
  err = opus_encoder_ctl(Enc, OPUS_SET_BITRATE(bit_rate));

  if (err != OPUS_OK)
  {
    while(1);
  }

  /*Complexity set*/
  err = opus_encoder_ctl(Enc, OPUS_SET_COMPLEXITY(0));

  if (err != OPUS_OK)
  {
    while(1);
  }

#if 0
  //Decoder Create.
  Dec = opus_decoder_create(sampling_rate, channels, &err);

  if (err != OPUS_OK)
  {
    while(1);
  }
#endif

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
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_DFSDM1_Init();

  /* USER CODE BEGIN 2 */
  //Enable SPI1.
  __HAL_SPI_ENABLE(&hspi1);

  //Enable Timer7.
  HAL_TIM_Base_Start_IT(&htim7);

  //Microphone
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0,
                                   &(Audio.Mic_Left_Channel_Buffer[0]),
                                   Audio_Buffer_In_Size
                                  );
/*
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1,
                                   &(Audio.Mic_Right_Channel_Buffer[0]),
                                   Audio_Buffer_In_Size
                                  );
*/
  //I2S Transmit Start.
  //HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*)Audio.Audio_Buffer_Out, Audio_Buffer_Out_Size);
#if 1
  /* Spirit1 Basic Mode Init */
  SPIRIT1_Basic_Init(&Spirit1,&Com);
  Test_Init();
#endif

  /* Matrix Keyboard Init */
  matrix_keyport_init(&matrix_key_data);

  /* STA350BW Init */
  STA350BW_Init(&STA350BW_I2C_Drv,  //SOUNDTERMINAL_Object_t* pObj,
                0x11,               //uint16_t                volume,
                STA350BW_Fs_48000,  //uint32_t                samplingFreq,
                NULL                //void*                   p
               );
#if 0
  //HAL_Delay(1000);

  /* Randam Numbers.

  srand(HAL_GetTick());

  for(Count = 0; Count < Audio_Buffer_In_Size; Count ++)
  {
    Random_Data[Count] = (int16_t)((rand() % 32768) * sin(HAL_GetTick()));
  }
  */
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*Matrix Keyboard.*/
    matrix_key_scan(&matrix_key_data);

#if Audio_Main_Loop == 1

    //Filter0 complete interrupt.
    if(Flag_DFSDM_Int[2])
    {
      Flag_DFSDM_Int[2] = 0;
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);

#if 1 /*Encode Audio.*/
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
      Count = opus_encode(Enc,
                          (opus_int16*)&Audio.Mic_Left_Channel_Buffer[0],
                          Audio_Buffer_In_Size,
                          (unsigned char*)&Audio.Audio_Buffer_Out_Encode[0][0],
                          Audio_Buffer_In_Size
                         );

      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
#endif
    }

#endif

#if 0
    /* If Key Record Press Down.
    */
    if(Audio_KEY_Get() == GPIO_PIN_RESET)
    {
      //Pause Speaker DMA.
      HAL_SAI_DMAPause(&hsai_BlockA2);

    }
    else
    {
      //Resume Speaker DMA.
      HAL_SAI_DMAResume(&hsai_BlockA2);
    }
#endif

    //Com_Main_Loop(&Com);

    //RX
    Test_Receive_Buffer();

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

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI2;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_SYSCLK;
  PeriphClkInit.PLLSAI2.PLLSAI2Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI2.PLLSAI2M = 1;
  PeriphClkInit.PLLSAI2.PLLSAI2N = 86;
  PeriphClkInit.PLLSAI2.PLLSAI2P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI2.PLLSAI2R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI2.PLLSAI2ClockOut = RCC_PLLSAI2_SAI2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the main internal regulator output voltage
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

/* DFSDM1 init function */
static void MX_DFSDM1_Init(void)
{

  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 22;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 3;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }

  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
  hdfsdm1_filter1.Init.FilterParam.Oversampling = 22;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 3;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    Error_Handler();
  }

  hdfsdm1_channel6.Instance = DFSDM1_Channel6;
  hdfsdm1_channel6.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel6.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel6.Init.OutputClock.Divider = 24;
  hdfsdm1_channel6.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel6.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel6.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel6.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel6.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel6.Init.Awd.FilterOrder = DFSDM_CHANNEL_SINC1_ORDER;
  hdfsdm1_channel6.Init.Awd.Oversampling = 10;
  hdfsdm1_channel6.Init.Offset = 0;
  hdfsdm1_channel6.Init.RightBitShift = 8;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel6) != HAL_OK)
  {
    Error_Handler();
  }

  hdfsdm1_channel7.Instance = DFSDM1_Channel7;
  hdfsdm1_channel7.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel7.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel7.Init.OutputClock.Divider = 24;
  hdfsdm1_channel7.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel7.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel7.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel7.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm1_channel7.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel7.Init.Awd.FilterOrder = DFSDM_CHANNEL_SINC1_ORDER;
  hdfsdm1_channel7.Init.Awd.Oversampling = 10;
  hdfsdm1_channel7.Init.Offset = 0;
  hdfsdm1_channel7.Init.RightBitShift = 8;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel7) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_6, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_7, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }

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
  hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
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
  htim7.Init.Period = 10000;
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

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PC13 Key_Line_In0_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|Key_Line_In0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Spirit1_GPIO0_Pin Spirit1_GPIO3_Pin */
  GPIO_InitStruct.Pin = Spirit1_GPIO0_Pin|Spirit1_GPIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STA350BW_PWRDN_Pin PA8 PA10 Debug2_Pin
                           Debug3_Pin */
  GPIO_InitStruct.Pin = STA350BW_PWRDN_Pin|GPIO_PIN_8|GPIO_PIN_10|Debug2_Pin
                          |Debug3_Pin;
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

  /*Configure GPIO pins : Debug0_Pin Debug1_Pin Key_Line_Out0_Pin Key_Line_Out1_Pin */
  GPIO_InitStruct.Pin = Debug0_Pin|Debug1_Pin|Key_Line_Out0_Pin|Key_Line_Out1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Key_Line_In1_Pin */
  GPIO_InitStruct.Pin = Key_Line_In1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Key_Line_In1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Spirit1_LED_Pin Spirit1_CSN_Pin */
  GPIO_InitStruct.Pin = Spirit1_LED_Pin|Spirit1_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STA350BW_PWRDN_Pin|GPIO_PIN_8|GPIO_PIN_10|Debug2_Pin
                          |Debug3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Debug0_Pin|Debug1_Pin|Key_Line_Out0_Pin|Key_Line_Out1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Spirit1_LED_Pin|Spirit1_CSN_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */


/* Timer Period Interrupt.
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim6)
  {
    if(Com.Flag_Receive_Buffer_Vaild == false)
    {
      //Com.Flag_Buffer_Vaild = true;

      //Com_Receive_Frame_IRQ(&Com);
    }
  }

  //Timer7 Period Interrupt.
  if(htim == &htim7)
  {
    matrix_key_data.flag_scan = true;
  }
}

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
