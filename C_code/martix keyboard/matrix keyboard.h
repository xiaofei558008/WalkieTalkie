/*This is type define which is used in matrix keyboard.c.
File name : matrix keyboard.h
All rights reserved,if the code is not authorized by STMicroelectronic.
2016-09-01 15:22:38 Tom.xiao@st.com
*/

#ifndef __MATRIX_KEY_BOARD
#define __MATRIX_KEY_BOARD

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "stm32l4xx_hal.h"
#include "matrix keyboard_CallBack.h"

/* keyboard and key scan parameters define;
*/
//the scale of keyboard is row_num * column_num;
//define row lines number;
#define row_num                 2
//define column lines number;
#define column_num              2

#define key_control_time        5
#define key_pressdown_time      15  //if the key pressdown longer then 15*key_pressdown_time ms
                                    //then it regard as keeping pressdown mode;

//after key_check_time,then the software get the key num(time >= (key_ckeck_time+1)*10)ms);
#define key_check_time          5

//GPIO Input Pins Define.
#define In_Pin0_Port            GPIOC
#define In_Pin0_Pin             GPIO_PIN_11
#define In_Pin0_Pin_Num         11

#define In_Pin1_Port            GPIOD
#define In_Pin1_Pin             GPIO_PIN_2
#define In_Pin1_Pin_Num         2

//GPIO Output Pins Define.
#define Out_Pin0_Port           GPIOC
#define Out_Pin0_Pin            GPIO_PIN_10
#define Out_Pin0_Pin_Num        10

#define Out_Pin1_Port           GPIOC
#define Out_Pin1_Pin            GPIO_PIN_12
#define Out_Pin1_Pin_Num        12

/*Below is the key in and out pins definition;
*/
//input define(if the input pins is not used,please define it as "1");
#define Pin0_in                 HAL_GPIO_ReadPin(In_Pin0_Port, In_Pin0_Pin)
#define Pin1_in                 HAL_GPIO_ReadPin(In_Pin1_Port, In_Pin1_Pin)
//#define Pin0_in                 ((In_Pin0_Port->IDR >> In_Pin0_Pin_Num) & 0x01)
//#define Pin1_in                 ((In_Pin1_Port->IDR >> In_Pin1_Pin_Num) & 0x01)
#define Pin2_in                 1
#define Pin3_in                 1
#define Pin4_in                 1
#define Pin5_in                 1
#define Pin6_in                 1
#define Pin7_in                 1
#define Pin8_in                 1
#define Pin9_in                 1
#define Pin10_in                1
#define Pin11_in                1
#define Pin12_in                1
#define Pin13_in                1
#define Pin14_in                1
#define Pin15_in                1

//output define(if the output pins is not used,please define it as "__asm("NOP")")
#define Pin0_out0               HAL_GPIO_WritePin(Out_Pin0_Port, Out_Pin0_Pin, GPIO_PIN_RESET)
#define Pin1_out0               HAL_GPIO_WritePin(Out_Pin1_Port, Out_Pin1_Pin, GPIO_PIN_RESET)
//#define Pin0_out0               (Out_Pin0_Port->ODR &= (uint16_t)~(1 << Out_Pin0_Pin_Num))
//#define Pin1_out0               (Out_Pin1_Port->ODR &= (uint16_t)~(1 << Out_Pin1_Pin_Num))
#define Pin2_out0               __asm("NOP")
#define Pin3_out0               __asm("NOP")
#define Pin4_out0               __asm("NOP")
#define Pin5_out0               __asm("NOP")
#define Pin6_out0               __asm("NOP")
#define Pin7_out0               __asm("NOP")
#define Pin8_out0               __asm("NOP")
#define Pin9_out0               __asm("NOP")
#define Pin10_out0              __asm("NOP")
#define Pin11_out0              __asm("NOP")
#define Pin12_out0              __asm("NOP")
#define Pin13_out0              __asm("NOP")
#define Pin14_out0              __asm("NOP")
#define Pin15_out0              __asm("NOP")

#define Pin0_out1               HAL_GPIO_WritePin(Out_Pin0_Port, Out_Pin0_Pin, GPIO_PIN_SET)
#define Pin1_out1               HAL_GPIO_WritePin(Out_Pin1_Port, Out_Pin1_Pin, GPIO_PIN_SET)
//#define Pin0_out1               (Out_Pin0_Port->ODR |= (1 << In_Pin0_Pin_Num))
//#define Pin1_out1               (Out_Pin1_Port->ODR |= (1 << In_Pin1_Pin_Num))
#define Pin2_out1               __asm("NOP")
#define Pin3_out1               __asm("NOP")
#define Pin4_out1               __asm("NOP")
#define Pin5_out1               __asm("NOP")
#define Pin6_out1               __asm("NOP")
#define Pin7_out1               __asm("NOP")
#define Pin8_out1               __asm("NOP")
#define Pin9_out1               __asm("NOP")
#define Pin10_out1              __asm("NOP")
#define Pin11_out1              __asm("NOP")
#define Pin12_out1              __asm("NOP")
#define Pin13_out1              __asm("NOP")
#define Pin14_out1              __asm("NOP")
#define Pin15_out1              __asm("NOP")

/* keyboard struct
*/
typedef struct
{
  uint8_t bit0:1;
  uint8_t bit1:1;
  uint8_t bit2:1;
  uint8_t bit3:1;
  uint8_t bit4:1;
  uint8_t bit5:1;
  uint8_t bit6:1;
  uint8_t bit7:1;
  uint8_t bit8:1;
  uint8_t bit9:1;
  uint8_t bit10:1;
  uint8_t bit11:1;
  uint8_t bit12:1;
  uint8_t bit13:1;
  uint8_t bit14:1;
  uint8_t bit15:1;
}matrix_struct_bits16;

typedef union
{
  uint16_t             all;
  matrix_struct_bits16 flag;
}matrix_u16_Bit;

typedef struct
{
  bool           flag_scan;

  uint8_t        count;
  uint8_t        scan_count;
  uint8_t        same_count;
  uint8_t        down_count;      //key press down count;

  matrix_u16_Bit group[2][16];    //for shake cleaning,7--based on read in lines(column)A group.
  matrix_u16_Bit flag[16];        //final key flags,after shake time;

  uint16_t       num[2];          //use for shake cleaning,the absolute key num
  uint16_t       now;             //final key num,after shake time;
  uint16_t       down;            //key press down data;

  uint16_t       trigger;         //first time press down flag(=1);
}matrix_keyboard_TypeDef;

/* Variables of key structure definition;
*/
extern matrix_keyboard_TypeDef matrix_key_data;

/*functions declare;
*/
void matrix_keyport_init(matrix_keyboard_TypeDef* temp);
void matrix_key_scan(matrix_keyboard_TypeDef* key);
static void matrix_key_num_control(matrix_keyboard_TypeDef* key);

#endif

/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

