/*This is matrix keyboard function;
File name : matrix keyboard_CallBack.c
All rights reserved,if the code is not authorized by STMicroelectronic.
2016-09-01 15:27:30 Tom.xiao@st.com
*/

/*
max matrix keyboard net -- 16*16 256 keys;

              |     |     |     |     |     |     |     |     |
row0   PB10---1-----2-----3-----4-----5-----6-----7-----|-----|-----
              |     |     |     |     |     |     |     |     |
row1   PB9----8-----9-----10----11----12----13----14----|-----|-----
              |     |     |     |     |     |     |     |     |
row2   PB11---15----16----17----18----19----20----21----|-----|-----
              |     |     |     |     |     |     |     |     |
row3   PB12---22----23----24----25----26----27----28----|-----|-----
              |     |     |     |     |     |     |     |     |
row4   PD12---29----30----31----32----33----34----35----|-----|-----
              |     |     |     |     |     |     |     |     |
row5   PD0----36----37----38----39----40----41----42----|-----|-----
              |     |     |     |     |     |     |     |     |
row6   PD1----43----44----45----46----47----48----49----|-----|-----
.             |     |     |     |     |     |     |     |     |
.         ----|-----|-----|-----|-----|-----|-----|-----|-----|-----
.             |     |     |     |     |     |     |     |     |
row15     ----|-----|-----|-----|-----|-----|-----|-----|-----256---
             PA12  PA13  PA14  PE0   PE1   PE2   PE3                  (input/IRQ)
              |     |     |     |     |     |     |     |     |
            colu0 colu1 colu2 colu3 colu4 colu5 colu6  ...  colu15
*/

#include "matrix keyboard.h"
#include "matrix keyboard_CallBack.h"
#include "SPIRIT1.h"
#include "test.h"
#include "sta350bw.h"
/*
         O   1


        3         2


             4

*/
/*
 * Key-Board CallBack Functions.
*/

/*  KEY 1 Management.
*/
//Key1 Trigger Function.
void key1_trigger_callback(void)
{
  uint8_t Temp_Volum;

  //Read Back Volum Data.
  STA350BW_I2C_Read(STA350BW_ADDRESS_1, STA350BW_MVOL, &Temp_Volum);

  //Add Volum.
  if(Temp_Volum > 15)
  {
    Temp_Volum -= 16;
  }

  //Set Back into STA350BW.
  STA350BW_I2C_Write(STA350BW_ADDRESS_1, STA350BW_MVOL, Temp_Volum);
}

//Key1 Continue Press Down.
void key1_press_down_callback(void)
{
  uint8_t Temp_Volum;

  //Read Back Volum Data.
  STA350BW_I2C_Read(STA350BW_ADDRESS_1, STA350BW_MVOL, &Temp_Volum);

  //Add Volum.
  if(Temp_Volum > 4)
  {
    Temp_Volum -= 4;
  }

  //Set Back into STA350BW.
  STA350BW_I2C_Write(STA350BW_ADDRESS_1, STA350BW_MVOL, Temp_Volum);
}

/*  KEY 2 Management.
*/
//Key2 Trigger Function.
void key2_trigger_callback(void)
{


}

//Key2 Continue Press Down.
void key2_press_down_callback(void)
{


}

/*  KEY 3 Management.
*/
//Key3 Trigger Function.
void key3_trigger_callback(void)
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

  Spirit_Tx(&Spirit1,
            Buffer_Byte_Len,
            Send_Byte_Buffer
           );
}

//Key3 Continue Press Down.
void key3_press_down_callback(void)
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

  Spirit_Tx(&Spirit1,
            Buffer_Byte_Len,
            Send_Byte_Buffer
           );
}

/*  KEY 4 Management.
*/
//Key4 Trigger Function.
void key4_trigger_callback(void)
{
  uint8_t Temp_Volum;

  //Read Back Volum Data.
  STA350BW_I2C_Read(STA350BW_ADDRESS_1, STA350BW_MVOL, &Temp_Volum);

  //Minus Volum.
  if(Temp_Volum < 240)
  {
    Temp_Volum += 16;
  }

  //Set Back into STA350BW.
  STA350BW_I2C_Write(STA350BW_ADDRESS_1, STA350BW_MVOL, Temp_Volum);
}

//Key4 Continue Press Down.
void key4_press_down_callback(void)
{
  uint8_t Temp_Volum;

  //Read Back Volum Data.
  STA350BW_I2C_Read(STA350BW_ADDRESS_1, STA350BW_MVOL, &Temp_Volum);

  //Minus Volum.
  if(Temp_Volum < 252)
  {
    Temp_Volum += 4;
  }

  //Set Back into STA350BW.
  STA350BW_I2C_Write(STA350BW_ADDRESS_1, STA350BW_MVOL, Temp_Volum);
}


/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

