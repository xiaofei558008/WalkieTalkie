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
#include "stm32l4xx_hal.h"
#include "math.h"

//Devices' head include
#include "sta350bw.h"

//functions define.
#define Audio_Buffer_Channel_Num    2
#define Audio_Buffer_In_Size        480
#define Audio_Buffer_Out_Size       (Audio_Buffer_In_Size * Audio_Buffer_Channel_Num)

#define Audio_Key_Record_Port       GPIOC
#define Audio_Key_Record_Pin        GPIO_PIN_13
#define Audio_KEY_Get()             HAL_GPIO_ReadPin(Audio_Key_Record_Port, Audio_Key_Record_Pin)

#define SaturaLH(N, L, H)           (((N)<(L))?(L):(((N)>(H))?(H):(N)))

#define Audio_Main_Loop             1

//typedef audio struct.
typedef struct
{
  uint32_t Ch_Data_H16    : 16;
  uint32_t Ch_Data_L8     : 8;
  uint32_t Dummy0         : 3;
  uint32_t Reg_Ch_Pending : 1;
  uint32_t Dummy1         : 1;
  uint32_t Ch_Convert_Num : 3;
}MCU_PCM_Format_TypeDef;

typedef union
{
  MCU_PCM_Format_TypeDef Audio;
  int32_t                All;
}Audio_Microphone_Data_TypeDef;

typedef struct
{
  int32_t                       Mic_Left_Channel_Buffer[Audio_Buffer_In_Size];
  int32_t                       Mic_Right_Channel_Buffer[Audio_Buffer_In_Size];

  int16_t                       Audio_Buffer_Out[Audio_Buffer_Out_Size];
  int16_t                       Audio_Buffer_Out_Encode[2][Audio_Buffer_In_Size];
}Audio_Microphone_TypeDef;

/*Global declare.
*/
extern Audio_Microphone_TypeDef Audio;
extern const int16_t Fragment1[];


#endif

/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

