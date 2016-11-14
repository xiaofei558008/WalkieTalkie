/*This is the Audio functions.
File name :Audio.c
All rights reserved,if the code is not authorized by STMicroelectronics.
2016-09-08 10:59:53 Tom.xiao@st.com
*/

#include "Audio.h"
#include "opus.h"
#include "AudioSin.h"

extern OpusEncoder* Enc;
extern OpusDecoder* Dec;
extern uint8_t      Flag_DFSDM_Int[4];

/* Audio Buffer Define.
*/
Audio_EnDecode_TypeDef Audio;

extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
extern DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
extern DFSDM_Channel_HandleTypeDef hdfsdm1_channel6;
extern DFSDM_Channel_HandleTypeDef hdfsdm1_channel7;
extern DMA_HandleTypeDef hdma_dfsdm1_flt0;
extern DMA_HandleTypeDef hdma_dfsdm1_flt1;

extern I2C_HandleTypeDef hi2c1;

extern SAI_HandleTypeDef hsai_BlockA2;
extern DMA_HandleTypeDef hdma_sai2_a;

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

/* Audio Init.
*/
void Audio_Init(Audio_EnDecode_TypeDef* Temp)
{
  //Clear Mempry.
  memset(&Audio,                        //void *s,
         0,                             //int ch,
         sizeof(Audio_EnDecode_TypeDef) //size_t n
        );

  //Set Tail.
  Temp->Tail[0] = 0x55;
  Temp->Tail[1] = 0xaa;
  Temp->Tail[2] = 0x55;
  Temp->Tail[3] = 0xaa;
}

/******************************
 * Audio Encode and Send out.
*/
void Audio_Encode_Send(Audio_EnDecode_TypeDef* Temp)
{
  /*Encode Audio.*/
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

  #if Audio_Sin_Teat == 0
  Temp->Encode.Encode_Out_Len = opus_encode(Enc,
                                            (opus_int16*)&Temp->Encode.Source_Buffer[0],
                                            Audio_Buffer_Sample_Point,
                                            (unsigned char*)&Temp->Encode.Encode_Out_Buffer[0],
                                            Audio_Buffer_Max
                                           );
  #else
  Temp->Encode.Encode_Out_Len = opus_encode(Enc,
                                            (opus_int16*)Audio_Sin_Data,
                                            Audio_Buffer_Sample_Point,
                                            (unsigned char*)&Temp->Encode.Encode_Out_Buffer[0],
                                            Audio_Buffer_Max
                                           );
  #endif
  //Add Tail.
  *((uint8_t*)&Temp->Encode.Encode_Out_Buffer[0] + (Temp->Encode.Encode_Out_Len++)) = Temp->Tail[0];
  *((uint8_t*)&Temp->Encode.Encode_Out_Buffer[0] + (Temp->Encode.Encode_Out_Len++)) = Temp->Tail[1];
  *((uint8_t*)&Temp->Encode.Encode_Out_Buffer[0] + (Temp->Encode.Encode_Out_Len++)) = Temp->Tail[2];
  *((uint8_t*)&Temp->Encode.Encode_Out_Buffer[0] + (Temp->Encode.Encode_Out_Len++)) = Temp->Tail[3];

  //Add Zeros.
  memset(&Temp->Encode.Encode_Out_Buffer[Temp->Encode.Encode_Out_Len],      //void *s,
         0,                                                                 //int ch,
         sizeof(uint8_t) * (Audio_Buffer_Max - Temp->Encode.Encode_Out_Len) //size_t n
        );

  //SPI DMA Transmiting.
  #if 0
  Spirit_Tx(&Spirit1,
            Temp->Encode.Encode_Out_Len,
            (uint8_t*)&Temp->Encode.Encode_Out_Buffer[0]
           );
  #else
  HAL_UART_Transmit_DMA(&huart1,
                        (uint8_t*)&Temp->Encode.Encode_Out_Buffer[0],
                        Audio_Buffer_Max    //Temp->Encode.Encode_Out_Len
                       );
  #endif
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

/******************************
* Audio Receive & Decode.
*/
void Audio_Receive_Decode(Audio_EnDecode_TypeDef* Temp)
{
  /* Frame Tail Comparing */
  //0x08 first byte finding.
  if(Temp->Decode.Head_Flag == true)
  {
#if 1
    //Tail Comparing.
    if(Temp->Decode.Receive_Byte_Buffer[Temp->Decode.Scan_Count] == Temp->Tail[Temp->Tail_Count])
    {
      Temp->Tail_Count ++;

      //Tail Fitted.
      if(Temp->Tail_Count == Audio_Tail_Len)
      {
        //Buffer not overflow.
        if(Temp->Decode.Overflow_Flag == false)
        {
          //Get Frame Length. Audio_Buffer_Max
          Temp->Decode.Encode_Byte_Len  = Temp->Decode.Scan_Count + 1 - Temp->Decode.Decode_Start_Num;
          Temp->Decode.Encode_Byte_Len -= Audio_Tail_Len;

          //Copy into Frame Buffer.
          memcpy(&Temp->Decode.Frame_Buffer[0],                           //void*dest,
                 &Temp->Decode.Receive_Byte_Buffer[Temp->Decode.Decode_Start_Num], //const void *src,
                 Temp->Decode.Encode_Byte_Len                                         //size_t n
                );

          //Clear Buffer.
          memset(&Temp->Decode.Receive_Byte_Buffer[Temp->Decode.Decode_Start_Num], //void *s,
                 0,                                                                 //int ch,
                 sizeof(uint8_t) * (Temp->Decode.Encode_Byte_Len + Audio_Tail_Len)    //size_t n
                );
        }

        //Buffer overflowed.
        else //###########################################################
        {
          //Get Frame Length.
          Temp->Decode.Encode_Byte_Len  = Audio_Buffer_Max - Temp->Decode.Decode_Start_Num;

          //Copy into Frame Buffer.
          memcpy(&Temp->Decode.Frame_Buffer[0],                                     //void*dest,
                 &Temp->Decode.Receive_Byte_Buffer[Temp->Decode.Decode_Start_Num],  //const void *src,
                 Temp->Decode.Encode_Byte_Len                                       //size_t n
                );

          //Clear Buffer.
          memset(&Temp->Decode.Receive_Byte_Buffer[Temp->Decode.Decode_Start_Num],  //void *s,
                 0,                                                                 //int ch,
                 sizeof(uint8_t) * Temp->Decode.Encode_Byte_Len                     //size_t n
                );

          //Copy into Frame Buffer.
          memcpy(&Temp->Decode.Frame_Buffer[Temp->Decode.Encode_Byte_Len],          //void*dest,
                 &Temp->Decode.Receive_Byte_Buffer[0],                              //const void *src,
                 Temp->Decode.Scan_Count + 1                                        //size_t n
                );

          //Clear Buffer.
          memset(&Temp->Decode.Receive_Byte_Buffer[0],                              //void *s,
                 0,                                                                 //int ch,
                 sizeof(uint8_t) * Temp->Decode.Scan_Count + 1                      //size_t n
                );

          //Get Frame Length(overflow).
          Temp->Decode.Encode_Byte_Len += (Temp->Decode.Scan_Count + 1);
          Temp->Decode.Encode_Byte_Len -= Audio_Tail_Len;
        }

        //Clear Head-Flag & Overflow-Flag.
        Temp->Decode.Head_Flag       = false;
        Temp->Decode.Overflow_Flag   = false;
        Temp->Tail_Count             = 0;

        /******************************
        *  Decode the Encode bytes.
        */
        /*TEST*/
        HAL_UART_Transmit_DMA(&huart3,
                              (uint8_t*)&Temp->Decode.Frame_Buffer[0],
                              Temp->Decode.Encode_Byte_Len
                             );
        /*TEST*/

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);


        /* Decode Audio and Out put */
        if(Temp->Decode.Encode_Byte_Len > 15)
        {
          Temp->Decode.Encode_Byte_Len = 6;
        }

        Temp->Decode.Decode_Len = opus_decode(Dec,
                                             (const unsigned char*)&Temp->Decode.Frame_Buffer[0],
                                             Temp->Decode.Encode_Byte_Len,
                                             (opus_int16*)&Temp->Decode.Decode_Buffer[0],
                                             Audio_Buffer_Sample_Point,
                                             0
                                            );
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
#if 1
        /* Copy into 2 Channel I2S Buffer */
        if(Temp->Decode.Decode_Len == Audio_Buffer_Sample_Point)
        {
          for(Temp->Decode.Scan_Count = 0; Temp->Decode.Scan_Count < Audio_Buffer_Sample_Point; Temp->Decode.Scan_Count ++)
          {
            Temp->Decode.Out_Buffer[Temp->Decode.Scan_Count * 2]     = Temp->Decode.Decode_Buffer[Temp->Decode.Scan_Count];
            Temp->Decode.Out_Buffer[Temp->Decode.Scan_Count * 2 + 1] = Temp->Decode.Decode_Buffer[Temp->Decode.Scan_Count];
          }
        }
#endif
      }
    }
    else
    {
      Temp->Tail_Count = 0;
    }
#endif
  }

  //First byte = 0x88 or not.
  else if(Temp->Decode.Receive_Byte_Buffer[Temp->Decode.Scan_Count] == 0x88)
  {
    //Set Flag True.
    Temp->Decode.Head_Flag = true;

    //Stored Count Number.
    Temp->Decode.Decode_Start_Num = Temp->Decode.Scan_Count;

  }

  //Add Scan Count.
  Temp->Decode.Scan_Count ++;

  if(Temp->Decode.Scan_Count >= Audio_Buffer_Max)
  {
    Temp->Decode.Scan_Count -= Audio_Buffer_Max;

    //Judge Overflow.
    if(Temp->Decode.Head_Flag == true)
    {
      Temp->Decode.Overflow_Flag = true;
    }
  }
}

/* Audio from Microphone to Speaker Directly.
*/
void Audio_Direct_Loop(Audio_EnDecode_TypeDef* Temp)
{
#if 0
  //Filter0 Half interrupt.
  if(Flag_DFSDM_Int[0])
  {
    Flag_DFSDM_Int[0] = 0;

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

    for(Count = 0; Count < Audio_Buffer_In_Size / 2; Count++)
    {
      Temp->Audio_Buffer_Out[Count*2] = SaturaLH((Temp->Mic_Left_Channel_Buffer[Count] >> 8), -32768, 32767);
    }
  }

  //Filter1 Half interrupt.
  if(Flag_DFSDM_Int[1])
  {
    Flag_DFSDM_Int[1] = 0;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);

    for(Count = 0; Count < Audio_Buffer_In_Size / 2; Count++)
    {
      Temp->Audio_Buffer_Out[Count * 2 + 1] = SaturaLH((Temp->Mic_Right_Channel_Buffer[Count] >> 8), -32768, 32767);
    }
  }

  //Filter0 complete interrupt.
  if(Flag_DFSDM_Int[2])
  {
    Flag_DFSDM_Int[2] = 0;
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);

    for(Count = Audio_Buffer_In_Size / 2; Count < Audio_Buffer_In_Size; Count++)
    {
      Temp->Audio_Buffer_Out[Count * 2] = SaturaLH((Temp->Mic_Left_Channel_Buffer[Count] >> 8), -32768, 32767);
    }
  }

  //Filter1 complete interrupt.
  if(Flag_DFSDM_Int[3])
  {
    Flag_DFSDM_Int[3] = 0;
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);

    for(Count = Audio_Buffer_In_Size / 2; Count < Audio_Buffer_In_Size; Count++)
    {
      Temp->Audio_Buffer_Out[Count * 2 + 1] = SaturaLH((Temp->Mic_Right_Channel_Buffer[Count] >> 8), -32768, 32767);
    }
  }
#endif
}

#if Audio_Main_Loop == 1
/*
*  DFSDM DMA Half Complete CallBack.
*/
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  if(hdfsdm_filter == &hdfsdm1_filter0)
  {
    //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
    Flag_DFSDM_Int[0] = 1;
  }

  if(hdfsdm_filter == &hdfsdm1_filter1)
  {
    //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
    Flag_DFSDM_Int[1] = 1;
  }
}

/*
*  DFSDM DMA Complete CallBack.
*/
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  if(hdfsdm_filter == &hdfsdm1_filter0)
  {
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
    Flag_DFSDM_Int[2] = 1;
  }

  if(hdfsdm_filter == &hdfsdm1_filter1)
  {
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
    Flag_DFSDM_Int[3] = 1;
  }
}

#else
/*
*  DFSDM DMA Half Complete CallBack.
*/
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint16_t Count;

  if(hdfsdm_filter == &hdfsdm1_filter0)
  {
    for(Count = 0; Count < Audio_Buffer_In_Size / 2; Count++)
    {
      Temp->Audio_Buffer_Out[Count * 2] = SaturaLH((Temp->Mic_Left_Channel_Buffer[Count] >> 8), -32768, 32767);
    }
  }

  if(hdfsdm_filter == &hdfsdm1_filter1)
  {
    for(Count = 0; Count < Audio_Buffer_In_Size / 2; Count++)
    {
      Temp->Audio_Buffer_Out[Count * 2 + 1] = SaturaLH((Temp->Mic_Right_Channel_Buffer[Count] >> 8), -32768, 32767);
    }

    //Start Transmiting.
    //HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*)Temp->Audio_Buffer_Out, Audio_Buffer_In_Size);
  }
}

/*
*  DFSDM DMA Complete CallBack.
*/
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  uint16_t Count;

  if(hdfsdm_filter == &hdfsdm1_filter0)
  {
    for(Count = Audio_Buffer_In_Size / 2; Count < Audio_Buffer_In_Size; Count++)
    {
      Temp->Audio_Buffer_Out[Count * 2] = SaturaLH((Temp->Mic_Left_Channel_Buffer[Count] >> 8), -32768, 32767);
    }
  }

  if(hdfsdm_filter == &hdfsdm1_filter1)
  {
    for(Count = Audio_Buffer_In_Size / 2; Count < Audio_Buffer_In_Size; Count++)
    {
      Temp->Audio_Buffer_Out[Count * 2 + 1] = SaturaLH((Temp->Mic_Right_Channel_Buffer[Count] >> 8), -32768, 32767);
    }

    //Start Transmiting.
    //HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*)Temp->Audio_Buffer_Out, Audio_Buffer_Out_Size);
    //HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*)&Temp->Audio_Buffer_Out[Audio_Buffer_In_Size], Audio_Buffer_In_Size);
  }
}

#endif

uint8_t  Temp;
uint16_t Temp1;

/*SAI Send DMA Half Complete CallBack.*/
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if(hsai == &hsai_BlockA2)
  {
#if 0
    for(Temp = 0; Temp < Audio_Buffer_In_Size / 2; Temp++)
    {
      Temp->Audio_Buffer_Out[Temp * 2]     = Fragment1[Temp1];
      Temp->Audio_Buffer_Out[Temp * 2 + 1] = Fragment1[Temp1];

      Temp1 = (Temp1 + 1) % 14678;
    }
#endif
  }
}

/*SAI Send DMA Complete CallBack.*/
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if(hsai == &hsai_BlockA2)
  {
#if 0
    for(Temp = Audio_Buffer_In_Size / 2; Temp < Audio_Buffer_In_Size; Temp++)
    {
      Temp->Audio_Buffer_Out[Temp * 2]     = Fragment1[Temp1];
      Temp->Audio_Buffer_Out[Temp * 2 + 1] = Fragment1[Temp1];

      Temp1 = (Temp1 + 1) % 14678;
    }
#endif
  }
}

/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

