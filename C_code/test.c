/*This is the test functions.
File name :test.c
All rights reserved,if the code is not authorized by STMicroelectronics.
2016-06-21 15:13:58 Tom.xiao@st.com
*/

#include "test.h"
#include "SPIRIT1.h"

uint8_t Send_Byte_Buffer[96],
        Receive_Byte_Buffer[96];

/* Buffer Init.
*/
void Test_Init(void)
{
  uint8_t Temp;

  Spirit1.Tx.Len = Buffer_Byte_Len;

  for(Temp = 0; Temp < Buffer_Byte_Len; Temp ++)
  {
    Send_Byte_Buffer[Temp] = Temp;
  }
}

/* Send Data.
*/
void Test_Send_Buffer(uint8_t  Len,
                      uint8_t* Buffer
                     )
{ //TX Command.
  //SpiritCmdStrobeTx();

  //Send Buffer.

}

/* Receive Data.
*/
void Test_Receive_Buffer(void)
{
  uint8_t Temp;
  bool    Temp_Flag = true;

  //Receive Command.
  SpiritCmdStrobeRx();

  if(SpiritLinearFifoReadNumElementsRxFifo() == Buffer_Byte_Len)
  {
    Spirit1.Rx.Len = Buffer_Byte_Len;
    SpiritSpiReadLinearFifo(Spirit1.Rx.Len, Spirit1.Rx.Buffer);

    for(Temp = 0; Temp < Buffer_Byte_Len; Temp ++)
    {
      //if not equal.
      if(Spirit1.Rx.Buffer[Temp] != Spirit1.Tx.Buffer[Temp])
      {
        Temp_Flag = false;
        break;
      }
    }

    //Received Success.
    if(Temp_Flag == true)
    {
      //Toggle LED;
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    }

    //Clean Rx FIFO.
    SpiritCmdStrobeFlushRxFifo();
  }
}

/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

