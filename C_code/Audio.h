/*This is the Audio functions' head.
File name :Audio.h
All rights reserved,if the code is not authorized by STMicroelectronics.
2016-06-21 15:14:9 Tom.xiao@st.com
*/

#ifndef __AUDIO_H
#define __AUDIO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "stm32l4xx_hal.h"
#include "math.h"

//Devices' head include
#include "sta350bw.h"

//functions define.
#define Audio_Channel_Num               2
#define Audio_Buffer_Sample_Point       240
#define Audio_Buffer_Out_Point          (Audio_Buffer_Sample_Point * Audio_Channel_Num)

#define Audio_Key_Record_Port           GPIOC
#define Audio_Key_Record_Pin            GPIO_PIN_13
#define Audio_KEY_Get()                 HAL_GPIO_ReadPin(Audio_Key_Record_Port, Audio_Key_Record_Pin)

#define SaturaLH(N, L, H)               (((N)<(L))?(L):(((N)>(H))?(H):(N)))

#define Audio_Main_Loop                 0
#define Audio_Direct                    1
#define Audio_Sin_Teat                  1

/* Audio Wireless Define.
*/
#define Audio_Buffer_Max                20  //96
#define Audio_Tail_Len                  4

/* Encode TypeDef.
*/
typedef struct
{
  int32_t       Source_Buffer[Audio_Buffer_Sample_Point];
  uint8_t       Encode_Out_Len;
  unsigned char Encode_Out_Buffer[Audio_Buffer_Max];
}Audio_Encode_TypeDef;

/* Decode TypeDef.
*/
typedef struct
{
  bool     Head_Flag;
  bool     Overflow_Flag;
  uint8_t  Scan_Count;

  uint8_t  Decode_Start_Num;
  uint16_t Encode_Byte_Len;

  uint8_t  Receive_DMA_Buffer[Audio_Buffer_Max];
  uint8_t  Receive_Byte_Buffer[Audio_Buffer_Max];
  uint8_t  Frame_Buffer[Audio_Buffer_Max];

  int16_t  Decode_Buffer[Audio_Buffer_Sample_Point];
  uint16_t Decode_Len;

  int16_t  Out_Buffer[Audio_Buffer_Out_Point];
}Audio_Decode_TypeDef;

typedef struct
{
  uint8_t               Gap_Count;
  uint8_t               Tail_Count;
  uint8_t               Tail[Audio_Tail_Len];
  Audio_Encode_TypeDef  Encode;
  Audio_Decode_TypeDef  Decode;
}Audio_EnDecode_TypeDef;

/*Global declare.
*/
extern Audio_EnDecode_TypeDef Audio;
extern const int16_t Fragment1[];

void Audio_Init(Audio_EnDecode_TypeDef* Temp);
void Audio_Encode_Send(Audio_EnDecode_TypeDef* Temp);
void Audio_Receive_Decode(Audio_EnDecode_TypeDef* Temp);
void Audio_Direct_Loop(Audio_EnDecode_TypeDef* Temp);

#endif

/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

