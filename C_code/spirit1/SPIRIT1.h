/*This is the SPIRIT1 functions' head.
File name :SPIRIT1.h
All rights reserved,if the code is not authorized by STMicroelectronics.
2016-08-08 14:57:57 Tom.xiao@st.com
*/

#ifndef __SPIRIT1_H
#define __SPIRIT1_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

//Devices' head include
#include "stm32l4xx_hal.h"
#include "XiFi.h"

/* SPIRIT1 Heads*/
#include "SPIRIT_Regs.h"
#include "SPIRIT_Aes.h"
#include "SPIRIT_Calibration.h"
#include "SPIRIT_Commands.h"
#include "SPIRIT_Csma.h"
#include "SPIRIT_DirectRF.h"
#include "SPIRIT_General.h"
#include "SPIRIT_Gpio.h"
#include "SPIRIT_Irq.h"
#include "SPIRIT_Timer.h"
#include "SPIRIT_LinearFifo.h"
#include "SPIRIT_PktCommon.h"
#include "SPIRIT_PktBasic.h"
#include "SPIRIT_PktMbus.h"
#include "SPIRIT_PktStack.h"
#include "SPIRIT_Config.h"
#include "SPIRIT_Qi.h"
#include "SPIRIT_Radio.h"
#include "SPIRIT1.h"
#include "SPIRIT_Types.h"
#include "SPIRIT_Management.h"

/* SpiritStatus TypeDefine.
*/
typedef union
{
  SpiritStatus Struct;
  uint16_t     u16;
}SpiritStatus_Union_TypeDef;

/**
* @brief  WMBus Link Layer State Enum.
*/
typedef enum
{
  SPIRIT_STA_RX = 0,
  SPIRIT_STA_TX,
  SPIRIT_STA_IDLE = 0xFF
}Spirit_State_TypeDef;

/*
* @brief Rx & Tx Data Buffer.
*/
typedef struct
{
  uint16_t Len;
  uint8_t* Buffer;
}Spirit_Buffer_TypeDef;

/* @ Spirit Version Struct.
*/
typedef struct
{
  uint8_t PartNum;
  uint8_t Version;
}Spirit_Ver_TypeDef;

/*
* @Spirit TypeDef
*/
typedef struct
{
  bool                   Flag_Rx_Valid;
  bool                   Flag_Rx_TimeOut;
  bool                   Flag_Tx_Ready;
  bool                   Flag_Tx_Done;

  Spirit_State_TypeDef   State;
  volatile SpiritStatus* Status;
  Spirit_Buffer_TypeDef  Tx;
  Spirit_Buffer_TypeDef  Rx;

  Spirit_Ver_TypeDef     Info;
}Spirit_TypeDef;

/*Global declare.
*/
extern SPI_HandleTypeDef hspi1;
extern Spirit_TypeDef Spirit1;
extern uint8_t Spirit_Reg[255];

void SPIRIT1_Dealy(uint32_t Delay_Temp);
void SPIRIT1_Basic_Init(Spirit_TypeDef*       Spirit_Temp,
                        Com_Protocol_TypeDef* Com_Temp
                       );

SpiritStatus SpiritSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
SpiritStatus SpiritSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
SpiritStatus SpiritSpiCommandStrobes(uint8_t cCommandCode);
SpiritStatus SpiritSpiWriteLinearFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
SpiritStatus SpiritSpiReadLinearFifo(uint8_t cNbBytes, uint8_t* pcBuffer);
uint8_t SPI_Send_Receive_Byte(uint8_t Data_Send);

void SpiritEnterShutdown(void);
void SpiritExitShutdown(void);

void Spirit_Main_Loop(Spirit_TypeDef* Temp);
void Spirit_Rx(Spirit_TypeDef* Temp);
void Spirit_Tx(Spirit_TypeDef* Temp);
void Spirit_Idle(Spirit_TypeDef* Temp);

void Spirit_Read_All_Reg(uint8_t* Reg_Buffer);

#endif

/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

