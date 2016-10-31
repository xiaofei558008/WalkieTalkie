/*This is the XiFi communication protocol functions' head.
File name :XiFi.h
All rights reserved,if the code is not authorized by STMicroelectronics.
2016-08-29 11:52:18 Tom.xiao@st.com
*/

#ifndef  __XIFI_H
#define  __XIFI_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "def32.h"

extern uint8_t  Test0,
                Test1;

extern uint16_t Test2,
                Test3,
                Test4;

extern uint32_t Test5,
                Test6;

/* Protocol Send Mode Define.
*/
#define Prot_Mod_Send_Only              0
#define Prot_Mod_Send_Loop_Back         1
#define Prot_Mod_Loop_Back_Only         2

#define Prot_Mod_Send                   Prot_Mod_Send_Loop_Back

/* Protocol Defines.
*/
#define Prot_Guest_Addr_Amount          100
#define Prot_Block_Size_Max             1024
#define Prot_TimeOutms                  500
#define Prot_CRC_Code                   0xa001
#define Prot_Send_Buffer_Size           100
#define Prot_Receive_Buffer_Size        100

#define Prot_Frame_Head_Length          11
#define Prot_CRC_Code                   0xa001
#define Prot_ACK_TimeOut_Amount         10

/* Address Mapping.
*/
#define Com_Mapping_Sec0_Enable         1
#define Com_Mapping_Sec1_Enable         0
#define Com_Mapping_Sec2_Enable         0

#define Com_Mapping_Addr_Offset_Sec0    0
#define Com_Mapping_Addr_Offset_Sec1    1000
#define Com_Mapping_Addr_Offset_Sec2    2000

/* Example.
#define Com_Mapping_Sec0_Addr0          ((uint8_t*)(&Add))
#define Com_Mapping_Sec0_Addr0          ((uint8_t*)(&Add +1))
*/
//Section 0
#define Com_Mapping_Sec0_Addr0          ((uint8_t*)(&Test0))

#define Com_Mapping_Sec0_Addr1          ((uint8_t*)(&Test1))

#define Com_Mapping_Sec0_Addr2          ((uint8_t*)(&Test2) + 1)
#define Com_Mapping_Sec0_Addr3          ((uint8_t*)(&Test2))

#define Com_Mapping_Sec0_Addr4          ((uint8_t*)(&Test3) + 1)
#define Com_Mapping_Sec0_Addr5          ((uint8_t*)(&Test3))

#define Com_Mapping_Sec0_Addr6          ((uint8_t*)(&Test4) + 1)
#define Com_Mapping_Sec0_Addr7          ((uint8_t*)(&Test4))

#define Com_Mapping_Sec0_Addr8          ((uint8_t*)(&Test5) + 3)
#define Com_Mapping_Sec0_Addr9          ((uint8_t*)(&Test5) + 2)
#define Com_Mapping_Sec0_Addr10         ((uint8_t*)(&Test5) + 1)
#define Com_Mapping_Sec0_Addr11         ((uint8_t*)(&Test5))

#define Com_Mapping_Sec0_Addr12         ((uint8_t*)(&Test6) + 3)
#define Com_Mapping_Sec0_Addr13         ((uint8_t*)(&Test6) + 2)
#define Com_Mapping_Sec0_Addr14         ((uint8_t*)(&Test6) + 1)
#define Com_Mapping_Sec0_Addr15         ((uint8_t*)(&Test6))

#define Com_Mapping_Sec0_Addr16         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr17         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr18         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr19         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr20         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr21         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr22         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr23         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr24         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr25         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr26         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr27         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr28         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr29         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr30         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr31         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr32         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr33         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr34         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr35         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr36         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr37         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr38         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr39         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr40         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr41         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr42         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr43         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr44         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr45         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr46         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr47         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr48         ((uint8_t*)(&Com))
#define Com_Mapping_Sec0_Addr49         ((uint8_t*)(&Com))

//Section1
#define Com_Mapping_Sec1_Addr0          ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr1          ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr2          ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr3          ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr4          ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr5          ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr6          ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr7          ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr8          ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr9          ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr10         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr11         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr12         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr13         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr14         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr15         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr16         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr17         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr18         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr19         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr20         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr21         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr22         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr23         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr24         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr25         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr26         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr27         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr28         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr29         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr30         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr31         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr32         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr33         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr34         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr35         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr36         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr37         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr38         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr39         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr40         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr41         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr42         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr43         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr44         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr45         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr46         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr47         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr48         ((uint8_t*)(&Com))
#define Com_Mapping_Sec1_Addr49         ((uint8_t*)(&Com))

//Section2.
#define Com_Mapping_Sec2_Addr0          ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr1          ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr2          ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr3          ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr4          ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr5          ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr6          ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr7          ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr8          ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr9          ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr10         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr11         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr12         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr13         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr14         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr15         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr16         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr17         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr18         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr19         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr20         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr21         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr22         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr23         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr24         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr25         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr26         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr27         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr28         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr29         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr30         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr31         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr32         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr33         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr34         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr35         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr36         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr37         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr38         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr39         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr40         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr41         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr42         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr43         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr44         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr45         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr46         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr47         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr48         ((uint8_t*)(&Com))
#define Com_Mapping_Sec2_Addr49         ((uint8_t*)(&Com))

/* Type define Protocol Com Status.
*/
typedef enum
{
  Com_STA_Free = 0,
  Com_STA_Send,
  Com_STA_Receive,
  Com_STA_ACK_Wait,
  Com_STA_Repeater,
  Com_STA_Test_Rec_Read_Simu,
  Com_STA_Test_Rec_Write_Simu,
}Com_Status_TypeDef;

/* Protocol Commander Code Define.
*/
typedef enum
{
  Com_CMD_Read = 0,
  Com_CMD_Write,
  Com_CMD_ACK,
  Com_CMD_Del,
  Com_CMD_Encry,
  Com_CMD_Free,
}Com_CMD_TypeDef;

/* Type define protocol struct.
*/
typedef struct
{
  Com_Status_TypeDef    Com_State;
  u16_bit               Flag_Error;

  u16_u8u8              Host_Addr;
  u16_u8u8              Guest_Addr;
  u16_u8u8              Guest_Addr_List[Prot_Guest_Addr_Amount];
  u32_u8u8u8u8          Reg_Addr;

  Com_CMD_TypeDef       CMD;
  u16_u8u8              Len;

  bool                  Flag_Send_Buffer_Vaild;
  uint8_t               Send_Data_Buffer[Prot_Send_Buffer_Size];

  bool                  Flag_Receive_Buffer_Vaild;
  uint8_t               Receive_Data_Buffer[Prot_Receive_Buffer_Size];

  uint16_t              TimeOut_Count;
  TIM_TypeDef*          Com_TIM;
}Com_Protocol_TypeDef;

/* Global Declare.
*/
extern Com_Protocol_TypeDef Com;

void Com_Init(Com_Protocol_TypeDef* Temp);
void Com_Main_Loop(Com_Protocol_TypeDef* Temp);
void Com_Send_Frame(Com_Protocol_TypeDef* Temp);

void Com_Receive_Simulation_Read_Test(Com_Protocol_TypeDef* Temp);
void Com_Receive_Simulation_Write_Test(Com_Protocol_TypeDef* Temp);

void Com_Receive_Frame_IRQ(Com_Protocol_TypeDef* Temp);
void Com_Receive_Management(Com_Protocol_TypeDef* Temp);

void Com_Send_Byte(uint8_t Data);
uint8_t Com_Receive_Byte(void);

void Com_ACK_Send(Com_Protocol_TypeDef* Temp,
                  bool                  Flag
                 );

void Com_Free(Com_Protocol_TypeDef* Temp);
void Com_ACK_Wait(Com_Protocol_TypeDef* Temp);
void Com_Repeater(Com_Protocol_TypeDef* Temp);

void Com_UID_Encryption(uint32_t Cmd);
uint16_t Com_CRC16_Calculate(uint8_t* reg_addr,
                             uint16_t frame_length
                            );

static uint8_t* Com_Addr_Mapping(uint32_t Addr_Temp);
void Com_Frame_Send_Finish_CallBack(void);
void Com_Frame_Receive_Finish_CallBack(void);

static void Com_Read_CallBack(Com_Protocol_TypeDef* Temp);
static void Com_Write_CallBack(Com_Protocol_TypeDef* Temp);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif

/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

