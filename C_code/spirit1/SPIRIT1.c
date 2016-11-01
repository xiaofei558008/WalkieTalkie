/* This is the SPIRIT1 functions.
File name :SPIRIT1.c
All rights reserved,if the code is not authorized by STMicroelectronics.
2016-08-08 14:57:40 Tom.xiao@st.com
*/

#include "SPIRIT1.h"
#include "Audio.h"
#include "test.h"

/* Global Variable.
*/
Spirit_TypeDef Spirit1;

/* Pins Define.
*/
#define SPIRIT1_SPI                 SPI1

#define SPIRIT1_CSn_Port            GPIOB
#define SPIRIT1_CSn_Pin             GPIO_PIN_6

#define SPIRIT1_SDn_Port            GPIOA
#define SPIRIT1_SDn_Pin             GPIO_PIN_10

#define SPIRIT1_GPIO0_Port          GPIOC
#define SPIRIT1_GPIO0_Pin           GPIO_PIN_1

#define SPIRIT1_GPIO1_Port          GPIOB
#define SPIRIT1_GPIO1_Pin           GPIO_PIN_0

#define SPIRIT1_GPIO2_Port          GPIOA
#define SPIRIT1_GPIO2_Pin           GPIO_PIN_4

#define SPIRIT1_GPIO3_Port          GPIOC
#define SPIRIT1_GPIO3_Pin           GPIO_PIN_7

#define SPIRIT1_LED_Port            GPIOB
#define SPIRIT1_LED_Pin             GPIO_PIN_4

#define SPIRIT1_CS_Delay_Time       200

#define SPIRIT1_SPI_CS_H()          {SPIRIT1_Dealy(SPIRIT1_CS_Delay_Time);                                 \
                                     HAL_GPIO_WritePin(SPIRIT1_CSn_Port, SPIRIT1_CSn_Pin, GPIO_PIN_SET);   \
                                    }

#define SPIRIT1_SPI_CS_L()          {HAL_GPIO_WritePin(SPIRIT1_CSn_Port, SPIRIT1_CSn_Pin, GPIO_PIN_RESET); \
                                     SPIRIT1_Dealy(SPIRIT1_CS_Delay_Time);                                 \
                                    }

#define SPIRIT1_Header_Write        0x00
#define SPIRIT1_Header_Read         0x01
#define SPIRIT1_Header_CMD          0x80

/* GPIO On MCU Side.
*/
typedef enum
{
  GPIO_CSn_Port   = 0x00,
  GPIO_SDn_Port   = 0x01,
  GPIO_GPIO0_Port = 0x02,
  GPIO_GPIO1_Port = 0x03,
  GPIO_GPIO2_Port = 0x04,
  GPIO_GPIO3_Port = 0x05,
  GPIO_LED_Port   = 0x06,
}SPIRIT_MCU_GPIO_TypeDef;

GPIO_TypeDef* SPIRIT_MCU_PORT[7] =
{
  SPIRIT1_CSn_Port,
  SPIRIT1_SDn_Port,
  SPIRIT1_GPIO0_Port,
  SPIRIT1_GPIO1_Port,
  SPIRIT1_GPIO2_Port,
  SPIRIT1_GPIO3_Port,
  SPIRIT1_LED_Port
};

uint16_t SPIRIT_MCU_Pin[7] =
{
  SPIRIT1_CSn_Pin,
  SPIRIT1_SDn_Pin,
  SPIRIT1_GPIO0_Pin,
  SPIRIT1_GPIO1_Pin,
  SPIRIT1_GPIO2_Pin,
  SPIRIT1_GPIO3_Pin,
  SPIRIT1_LED_Pin
};

extern SpiritStatus Temp_S;
extern uint8_t Version[25];

/* SPIRIT1 Delay.
*/
void SPIRIT1_Dealy(uint32_t Delay_Temp)
{
  while(Delay_Temp --);
}

/* Init SPIRIT1.
*/
void SPIRIT1_Basic_Init(Spirit_TypeDef*       Spirit_Temp,
                        Com_Protocol_TypeDef* Com_Temp
                       )
{ //Init MCU GPIO.

  //Init MCU SPI.

  //Set CS High.
  SPIRIT1_SPI_CS_H();

  //Reset SPIRIT1.
  SpiritEnterShutdown();
  SPIRIT1_Dealy(300000);
  SpiritExitShutdown();
  SPIRIT1_Dealy(300000);

  /* Spirit1 Variables Init */
#if 0
  Spirit_Temp->Rx.Buffer = Com_Temp->Receive_Data_Buffer;
  Spirit_Temp->Tx.Buffer = Com_Temp->Send_Data_Buffer;

#elif 0
  Spirit_Temp->Rx.Buffer = Receive_Byte_Buffer;
  Spirit_Temp->Tx.Buffer = Send_Byte_Buffer;

#elif 1
  Spirit_Temp->Rx.Buffer = (uint8_t*)Audio.Audio_Buffer_In_Encode;
  Spirit_Temp->Tx.Buffer = (uint8_t*)Audio.Audio_Buffer_Out_Encode;

#endif

  Spirit_Temp->Status    = &g_xStatus;

  // Read Part Number & Version.
  *(Spirit1.Status) = SpiritSpiReadRegisters(DEVICE_INFO1_PARTNUM, 2, &Spirit1.Info.PartNum);

  //Spirit1 Init.
  SGpioInit GPIO_Temp;
  GPIO_Temp.xSpiritGpioPin  = SPIRIT_GPIO_3;
  GPIO_Temp.xSpiritGpioMode = SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP;
  GPIO_Temp.xSpiritGpioIO   = SPIRIT_GPIO_DIG_OUT_IRQ;
  SpiritGpioInit(&GPIO_Temp);

#if 0
  GPIO_Temp.xSpiritGpioPin  = SPIRIT_GPIO_2;
  SpiritGpioInit(&GPIO_Temp);

  GPIO_Temp.xSpiritGpioPin  = SPIRIT_GPIO_1;
  SpiritGpioInit(&GPIO_Temp);

  GPIO_Temp.xSpiritGpioPin  = SPIRIT_GPIO_0;
  SpiritGpioInit(&GPIO_Temp);
#endif

  //Set External Crystal = 24MHz.
  SpiritRadioSetXtalFlag(XTAL_FLAG_24_MHz);
  SpiritRadioSetXtalFrequency(50000000);

  //Radio Init.
  SRadioInit xRadioInit =
  {
    .nXtalOffsetPpm     = 0,        //XTAL_OFFSET_PPM,
    .lFrequencyBase     = 915.0e6,  //BASE_FREQUENCY,
    .nChannelSpace      = 20e3,     //CHANNEL_SPACE,
    .cChannelNumber     = 0,        //CHANNEL_NUMBER,
    .xModulationSelect  = FSK,      //MODULATION_SELECT,
    .lDatarate          = 50000,     //DATARATE,
    .lFreqDev           = 20e3,     //FREQ_DEVIATION,
    .lBandwidth         = 100E3     //BANDWIDTH
  };

  SpiritRadioInit(&xRadioInit);

  //Spirit Radio set power.
  SpiritRadioSetPALeveldBm(7, 11.6);
  SpiritRadioSetPALevelMaxIndex(7);

  //Packet Init, Spirit Basic Packet config.
  PktBasicInit xBasicInit =
  {
    .xPreambleLength = PKT_PREAMBLE_LENGTH_04BYTES, //PREAMBLE_LENGTH,
    .xSyncLength     = PKT_SYNC_LENGTH_4BYTES,      //SYNC_LENGTH,
    .lSyncWords      = 0x88888888,                  //SYNC_WORD,
    .xFixVarLength   = PKT_LENGTH_VAR,              //LENGTH_TYPE,
    .cPktLengthWidth = 7,                           //LENGTH_WIDTH,
    .xCrcMode        = PKT_CRC_MODE_8BITS,          //CRC_MODE,
    .xControlLength  = PKT_CONTROL_LENGTH_0BYTES,   //CONTROL_LENGTH,
    .xAddressField   = S_ENABLE,
    .xFec            = S_DISABLE,                   //EN_FEC,
    .xDataWhitening  = S_ENABLE                     //EN_WHITENING
  };

  SpiritPktBasicInit(&xBasicInit);

  //enable SQI check.
  SpiritQiSetSqiThreshold(SQI_TH_0);
  SpiritQiSqiCheck(S_ENABLE);

  //Set RSSI Threshold.
  SpiritQiSetRssiThresholddBm(-120);

  //Set Basic Address.
  PktBasicAddressesInit xAddressInit =
  {
    .xFilterOnMyAddress         = S_ENABLE,
    .cMyAddress                 = 0x44,
    .xFilterOnMulticastAddress  = S_DISABLE,
    .cMulticastAddress          = 0xee,
    .xFilterOnBroadcastAddress  = S_ENABLE,
    .cBroadcastAddress          = 0xff
  };

  SpiritPktBasicAddressesInit(&xAddressInit);

  /* Spirit IRQs disable */
  SpiritIrqDeInit(NULL);

  /* Spirit IRQs enable */
  //RX
  SpiritIrq(RX_DATA_READY, S_ENABLE);
  SpiritIrq(RX_DATA_DISC, S_ENABLE);
  SpiritIrq(RX_TIMEOUT, S_ENABLE);

  //TX
  SpiritIrq(TX_DATA_SENT, S_ENABLE);
  //SpiritIrq(MAX_RE_TX_REACH, S_ENABLE);

#if 0
  /* rx timeout config */
  if(RECEIVE_TIMEOUT == 0)
  {
    /* rx timeout config */
    SET_INFINITE_RX_TIMEOUT();
    SpiritTimerSetRxTimeoutStopCondition(ANY_ABOVE_THRESHOLD);
  }
  else
#endif
  {
    /* RX timeout config */
    SpiritTimerSetRxTimeoutMs(2000.0);

    /* enable SQI check */
    SpiritQiSetSqiThreshold(SQI_TH_0);
    SpiritQiSqiCheck(S_ENABLE);

    SpiritTimerSetRxTimeoutStopCondition(RSSI_AND_SQI_ABOVE_THRESHOLD);
  }

  /* destination address */
  SpiritPktBasicSetDestinationAddress(0x44);

  /* IRQ registers blanking */
  SpiritIrqClearStatus();

  /* RX command */
  if(g_xStatus.MC_STATE != MC_STATE_RX)
  {
    /* Exit to Ready Mode */
    SpiritCmdStrobeSabort();

    /* RX command */
    SpiritCmdStrobeRx();
  }
}

/* SPIRIT1 Send Data.
*/
SpiritStatus SpiritSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t  Temp;
  SpiritStatus_Union_TypeDef Status;

  //CS Low
  SPIRIT1_SPI_CS_L();

  //Send Write Header.
  Status.u16 = SPI_Send_Receive_Byte(SPIRIT1_Header_Write);
  Status.u16 <<= 8;

  //Send Reg Address.
  Status.u16 |= SPI_Send_Receive_Byte(cRegAddress);

  //Write Datas.
  for(Temp = 0; Temp < cNbBytes; Temp ++)
  {
    SPI_Send_Receive_Byte(*(pcBuffer + Temp));
  }

  //CS High.
  SPIRIT1_SPI_CS_H();

  return(Status.Struct);
}

/* SPIRIT1 Receive Data.
*/
SpiritStatus SpiritSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t  Temp;
  SpiritStatus_Union_TypeDef Status;

  //CS Low
  SPIRIT1_SPI_CS_L();

  //Send Read Header.
  Status.u16 = SPI_Send_Receive_Byte(SPIRIT1_Header_Read);
  Status.u16 <<= 8;

  //Send Reg Address.
  Status.u16 |= SPI_Send_Receive_Byte(cRegAddress);

  //Read Datas.
  for(Temp = 0; Temp < cNbBytes; Temp ++)
  {
    *(pcBuffer + Temp) = SPI_Send_Receive_Byte(0x00);
  }

  //CS High.
  SPIRIT1_SPI_CS_H();

  return(Status.Struct);
}

/* SPIRIT1 Write Command.
*/
SpiritStatus SpiritSpiCommandStrobes(uint8_t cCommandCode)
{
  SpiritStatus_Union_TypeDef Status;

  //CS Low
  SPIRIT1_SPI_CS_L();

  //Send Command Header.
  Status.u16 = SPI_Send_Receive_Byte(SPIRIT1_Header_CMD);
  Status.u16 <<= 8;

  //Send Command.
  Status.u16 |= SPI_Send_Receive_Byte(cCommandCode);

  //CS High.
  SPIRIT1_SPI_CS_H();

  return(Status.Struct);
}

/* Write FIFO.
*/
SpiritStatus SpiritSpiWriteLinearFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  SpiritStatus Status;

  Status = SpiritSpiWriteRegisters(0xff, cNbBytes, pcBuffer);
  return Status;
}

/* Read FIFO.
*/
SpiritStatus SpiritSpiReadLinearFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
  SpiritStatus Status;

  Status = SpiritSpiReadRegisters(0xff, cNbBytes, pcBuffer);
  return Status;
}

/* SPI Send & Receive One Byte.
*/
uint8_t SPI_Send_Receive_Byte(uint8_t Data_Send)
{
  //Waiting for Transmiting Complete.
  while(SPIRIT1_SPI->SR & SPI_FLAG_BSY);

  *((__IO uint8_t*)((uint32_t)SPIRIT1_SPI + 0x0c)) = Data_Send;

  //Waiting for Receiving Complete.
  while(SPIRIT1_SPI->SR & SPI_FLAG_BSY);

  return *((__IO uint8_t*)((uint32_t)SPIRIT1_SPI + 0x0c));
}

/* SDN Set/Reset.
*/
void SpiritEnterShutdown(void)
{
  HAL_GPIO_WritePin(SPIRIT1_SDn_Port, SPIRIT1_SDn_Pin, GPIO_PIN_SET);
}

void SpiritExitShutdown(void)
{
  HAL_GPIO_WritePin(SPIRIT1_SDn_Port, SPIRIT1_SDn_Pin, GPIO_PIN_RESET);
}

/* Spirit Main Loop
*/
void Spirit_Main_Loop(Spirit_TypeDef* Temp)
{
  switch(Temp->State)
  {
    case SPIRIT_STA_RX:   //Spirit_Rx(Temp);
    break;

    case SPIRIT_STA_TX:   //Spirit_Tx(Temp);
    break;

    //case SPIRIT_STA_IDLE: Spirit_Idle();
    //break;
  }
}

/* Spirit Receive Start.
*/
uint8_t Spirit_Rx(Spirit_TypeDef* Temp,
                  uint8_t*        Byte_Buffer
                 )
{ /* Set up Buffer */
  Temp->Rx.Buffer = Byte_Buffer;

  //Read FiFO
  Spirit1.Rx.Len = SpiritLinearFifoReadNumElementsRxFifo();
  SpiritSpiReadLinearFifo(Spirit1.Rx.Len, Spirit1.Rx.Buffer);

  //Clean Rx FIFO.
  //SpiritCmdStrobeFlushRxFifo()

#if 1
  /* RX command */
  if(g_xStatus.MC_STATE != MC_STATE_RX)
  {
    /* Exit to Ready Mode */
    SpiritCmdStrobeSabort();

    /* RX command */
    SpiritCmdStrobeRx();
  }
#endif

  return Spirit1.Rx.Len;
}

/* Spirit Send Start.
*/
void Spirit_Tx(Spirit_TypeDef* Temp,
               uint8_t         Byte_Len,
               uint8_t*        Byte_Buffer
              )
{ /* Set up Spirit1 Buffer */
  Temp->Tx.Len = Byte_Len;
  Temp->Tx.Buffer = Byte_Buffer;

  /* IRQ registers blanking */
  SpiritIrqClearStatus();

  /* ready state command */
  SpiritCmdStrobeSabort();

  /* Clean the TX FIFO. */
  SpiritCmdStrobeFlushTxFifo();

  /* payload length config */
  SpiritPktBasicSetPayloadLength(Temp->Tx.Len);

  /* Write Tx FIFO */
  SpiritSpiWriteLinearFifo(Temp->Tx.Len, Temp->Tx.Buffer);

  /* send the TX command */
  SpiritCmdStrobeTx();

  /* Waiting for Transmiting Finished */
  //while(SpiritLinearFifoReadNumElementsTxFifo() != 0);
}

/* Spirit Idle
*/
void Spirit_Idle(Spirit_TypeDef* Temp)
{

}

/* Read All Registers.
*/
uint8_t Spirit_Reg[255];

void Spirit_Read_All_Reg(uint8_t* Reg_Buffer)
{
  uint8_t Temp;

  for(Temp = 0; Temp < 0xff; Temp ++)
  {
    SpiritSpiReadRegisters(0x00, 0xff, Reg_Buffer);
  }
}



/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

