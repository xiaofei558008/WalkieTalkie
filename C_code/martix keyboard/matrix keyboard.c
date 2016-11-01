/*This is matrix keyboard function;
File name : matrix keyboard.c
All rights reserved,if the code is not authorized by STMicroelectronic.
----by xiaofei
E-mail:tom.xiao@st.com
2013-4-19 18:08:10
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

/* Variables of key structure definition;
*/
matrix_keyboard_TypeDef matrix_key_data;   //define the key data structure;

/*init key ports,which is defined on MCU GPIO;
*/
void matrix_keyport_init(matrix_keyboard_TypeDef* temp)
{
  //GPIO Init.

  //Init Key Variables.
  temp->scan_count = 0;
  temp->same_count = 0;
  temp->down_count = 0;

  temp->num[0]     = 0;
  temp->num[1]     = 0;
  temp->now        = 0;
  temp->down       = 0;
  temp->trigger    = 0;
}

/* managed in 10ms ,and then return key;
   parameter:
   matrix_keyboard_TypeDef* key ---- from global key variable definition;
   u1_bit flag ---- the time flag,which comes from sys ticks(10ms flag recommend);
   The flag of bit0 is used in this function,if you want change the bit,
   you can change line110 and line 112;
*/
void matrix_key_scan(matrix_keyboard_TypeDef* key)
{ uint8_t        temp,flag_odd_even;
  uint16_t       key_temp=0,key_base;
  matrix_u16_Bit scan_data[16] = {0xfffe,0xfffd,0xfffb,0xfff7,
                                  0xffef,0xffdf,0xffbf,0xff7f,
                                  0xfeff,0xfdff,0xfbff,0xf7ff,
                                  0xefff,0xdfff,0xbfff,0x7fff,
                                 };
  //from sys tick(10ms);
  if(key->flag_scan)
  {
    //Reset Flag.
    key->flag_scan = false;

    flag_odd_even = key->scan_count & 0x01;

    key->scan_count++;      //global counter only need last bit,so there is no need to clean 0;

    for(temp = 0; temp < row_num; temp ++)
    {
      /*send row data
      */
      if(scan_data[temp].flag.bit0)  {Pin0_out1;}  else {Pin0_out0;}
      if(scan_data[temp].flag.bit1)  {Pin1_out1;}  else {Pin1_out0;}
      if(scan_data[temp].flag.bit2)  {Pin2_out1;}  else {Pin2_out0;}
      if(scan_data[temp].flag.bit3)  {Pin3_out1;}  else {Pin3_out0;}
      if(scan_data[temp].flag.bit4)  {Pin4_out1;}  else {Pin4_out0;}
      if(scan_data[temp].flag.bit5)  {Pin5_out1;}  else {Pin5_out0;}
      if(scan_data[temp].flag.bit6)  {Pin6_out1;}  else {Pin6_out0;}
      if(scan_data[temp].flag.bit7)  {Pin7_out1;}  else {Pin7_out0;}
      if(scan_data[temp].flag.bit8)  {Pin8_out1;}  else {Pin8_out0;}
      if(scan_data[temp].flag.bit9)  {Pin9_out1;}  else {Pin9_out0;}
      if(scan_data[temp].flag.bit10) {Pin10_out1;} else {Pin10_out0;}
      if(scan_data[temp].flag.bit11) {Pin11_out1;} else {Pin11_out0;}
      if(scan_data[temp].flag.bit12) {Pin12_out1;} else {Pin12_out0;}
      if(scan_data[temp].flag.bit13) {Pin13_out1;} else {Pin13_out0;}
      if(scan_data[temp].flag.bit14) {Pin14_out1;} else {Pin14_out0;}
      if(scan_data[temp].flag.bit15) {Pin15_out1;} else {Pin15_out0;}

      key_base = temp * column_num;   //get current key base data;

      /*get column data
      */
      if(Pin0_in)
      {
        key->group[flag_odd_even][temp].flag.bit0 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit0 = 1;
        key_temp = key_base + 1;
      }
      //...
      if(Pin1_in)
      {
        key->group[flag_odd_even][temp].flag.bit1 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit1 = 1;
        key_temp = key_base + 2;
      }
      //...
      if(Pin2_in)
      {
        key->group[flag_odd_even][temp].flag.bit2 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit2 = 1;
        key_temp = key_base + 3;
      }
      //...
      if(Pin3_in)
      {
        key->group[flag_odd_even][temp].flag.bit3 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit3 = 1;
        key_temp = key_base + 4;
      }
      //...
      if(Pin4_in)
      {
        key->group[flag_odd_even][temp].flag.bit4 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit4 = 1;
        key_temp = key_base + 5;
      }
      //...
      if(Pin5_in)
      {
        key->group[flag_odd_even][temp].flag.bit5 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit5 = 1;
        key_temp = key_base + 6;
      }
      //...
      if(Pin6_in)
      {
        key->group[flag_odd_even][temp].flag.bit6 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit6 = 1;
        key_temp = key_base + 7;
      }
      if(Pin7_in)
      {
        key->group[flag_odd_even][temp].flag.bit7 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit7 = 1;
        key_temp = key_base + 8;
      }
      //...
      if(Pin8_in)
      {
        key->group[flag_odd_even][temp].flag.bit8 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit8 = 1;
        key_temp = key_base + 9;
      }
      //...
      if(Pin9_in)
      {
        key->group[flag_odd_even][temp].flag.bit9 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit9 = 1;
        key_temp = key_base + 10;
      }
      //...
      if(Pin10_in)
      {
        key->group[flag_odd_even][temp].flag.bit10 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit10 = 1;
        key_temp = key_base + 11;
      }
      //...
      if(Pin11_in)
      {
        key->group[flag_odd_even][temp].flag.bit11 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit11 = 1;
        key_temp = key_base + 12;
      }
      //...
      if(Pin12_in)
      {
        key->group[flag_odd_even][temp].flag.bit12 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit12 = 1;
        key_temp = key_base + 13;
      }
      //...
      if(Pin13_in)
      {
        key->group[flag_odd_even][temp].flag.bit13 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit13 = 1;
        key_temp = key_base + 14;
      }
      if(Pin14_in)
      {
        key->group[flag_odd_even][temp].flag.bit14 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit14 = 1;
        key_temp = key_base + 15;
      }
      //...
      if(Pin15_in)
      {
        key->group[flag_odd_even][temp].flag.bit15 = 0;
      }
      else
      {
        key->group[flag_odd_even][temp].flag.bit15 = 1;
        key_temp = key_base + 16;
      }
    }//end of /for(temp=0;temp<8;temp++)

    key->num[flag_odd_even] = key_temp;

    /* key shaking cleaning;
    */
    if((key->group[1][0].all == key->group[0][0].all)&&
       (key->group[1][1].all == key->group[0][1].all)&&
       (key->group[1][2].all == key->group[0][2].all)&&
       (key->group[1][3].all == key->group[0][3].all)&&
       (key->group[1][4].all == key->group[0][4].all)&&
       (key->group[1][5].all == key->group[0][5].all)&&
       (key->group[1][6].all == key->group[0][6].all)&&
       (key->num[1] == key->num[0])
      )
    {
      key->same_count++;
      if(key->same_count > key_check_time)    //same > 5 shaking cleaning successfully,50ms refresh,the key state;
      {
        key->same_count = 0;

        key->flag[0].all = key->group[flag_odd_even][0].all;  //final key state read out;
        key->flag[1].all = key->group[flag_odd_even][1].all;
        key->flag[2].all = key->group[flag_odd_even][2].all;
        key->flag[3].all = key->group[flag_odd_even][3].all;
        key->flag[4].all = key->group[flag_odd_even][4].all;
        key->flag[5].all = key->group[flag_odd_even][5].all;
        key->flag[6].all = key->group[flag_odd_even][6].all;

        key->now = key->num[flag_odd_even];                   //final key abs read out;
        key->trigger = key->now & (key->now ^ key->down);
        key->down = key->now;
      }
    }//end of key state same judge.
    else                    //not same then clean the same count.
    {
      key->same_count=0;
    }//end of key state not same judge.

    /*
    *  @key number control part.
    */
    key->count ++;
    if(key->count >= key_control_time)
    {
      key->count = 0;

      //key number controled every 50ms.
      matrix_key_num_control(key);
    }
  }//end of /if(flag_10ms)
}

/* This function including every key num functions,user can add their own code in those functions;
   key num managed in 50ms.Recommend offset 25ms,as time flag used in key_scan;
   parameter:
   matrix_keyboard_TypeDef* key ---- from global key variable definition;
   The time flag,which comes from sys ticks(50ms with 25ms offset flag recommend);
*/
static void matrix_key_num_control(matrix_keyboard_TypeDef* key)
{
  /* Mult-key manage:user can add any keys together defining a function;
     Only 1 bit can also be idea;
     Advice that,keeping a main key press down firstly,which is no used in 1 bit idea,then press the other keys;
     Blow giving an idea,which combining 2 key together;
     Other bits combining can be defined by user freely;
  */

  /*  R1C1+R1C2 key
  */
  if(key->flag[0].flag.bit0 &&    //key in row 1 column 1;
     key->flag[0].flag.bit1       //key in row 1 column 2;
    )
  {
    if(key->trigger)              //press the key down first time;
    {
      key->trigger = 0;
      key->down_count = 0;

      // user add the one time mode code below;

    }
    else
    {
      key->down_count ++;
      if(key->down_count >= key_pressdown_time)
      {
        key->down_count = key_pressdown_time;

        // user add the keeping pressdown mode code below;

      }
    }
  }
                                                        //++++ R1C1 & R2C1 ++++
  /* User can add union keys below.
  */


  /* Absolute key data manage,which is contaning 1-49 key num,0 is read as no key press down;
     user can manage the one press mode and keeping down mode in below switch structure;
  */
  switch(key->now)
  {
    case 0 :                                            //###### KEY 0 ###### (no key press down)
    {

    }break;   //no key press down;
    case 1 :                                            //###### KEY 1 ######
    {
      if(key->trigger)    //press the key down first time;
      {
        key->trigger = 0;
        key->down_count = 0;

        // user add the one time mode code below;
        key1_trigger_callback();
      }
      else
      {
        key->down_count ++;
        if(key->down_count >= key_pressdown_time)
        { key->down_count = key_pressdown_time;

          // user add the continues press down mode code below;
          key1_press_down_callback();
        }
      }
    }break;
    case 2 :                                            //###### KEY 2 ######
    {
      if(key->trigger)    //press the key down first time;
      {
        key->trigger = 0;
        key->down_count = 0;

        // user add the one time mode code below;
        key2_trigger_callback();
      }
      else
      {
        key->down_count ++;
        if(key->down_count >= key_pressdown_time)
        { key->down_count = key_pressdown_time;

          // user add the continues press down mode code below;
          key2_press_down_callback();
        }
      }
    }break;
    case 3 :                                            //###### KEY 3 ######
    {
      if(key->trigger)    //press the key down first time;
      {
        key->trigger = 0;
        key->down_count = 0;

        // user add the one time mode code below;
        key3_trigger_callback();
      }
      else
      {
        key->down_count ++;
        if(key->down_count >= key_pressdown_time)
        { key->down_count = key_pressdown_time;

          // user add the continues press down mode code below;
          key3_press_down_callback();
        }
      }
    }break;
    case 4 :                                            //###### KEY 4 ######
    {
      if(key->trigger)    //press the key down first time;
      {
        key->trigger = 0;
        key->down_count = 0;

        // user add the one time mode code below;
        key4_trigger_callback();
      }
      else
      {
        key->down_count ++;
        if(key->down_count >= key_pressdown_time)
        { key->down_count = key_pressdown_time;

          // user add the continues press down mode code below;
          key4_press_down_callback();
        }
      }
    }break;
    case 5 :                                            //###### KEY 5 ######
    {
      if(key->trigger)    //press the key down first time;
      {
        key->trigger = 0;
        key->down_count = 0;
        // user add the one time mode code below;


      }
      else
      {
        key->down_count ++;
        if(key->down_count >= key_pressdown_time)
        { key->down_count = key_pressdown_time;
          // user add the one time mode code below;


        }
      }
    }break;
    case 6 :                                            //###### KEY 6 ######
    {
      if(key->trigger)    //press the key down first time;
      {
        key->trigger = 0;
        key->down_count = 0;
        // user add the one time mode code below;


      }
      else
      {
        key->down_count ++;
        if(key->down_count >= key_pressdown_time)
        { key->down_count = key_pressdown_time;
          // user add the one time mode code below;


        }
      }
    }break;
    case 7 :                                            //###### KEY 7 ######
    {
      if(key->trigger)    //press the key down first time;
      {
        key->trigger = 0;
        key->down_count = 0;
        // user add the one time mode code below;


      }
      else
      {
        key->down_count ++;
        if(key->down_count >= key_pressdown_time)
        { key->down_count = key_pressdown_time;
          // user add the one time mode code below;


        }
      }
    }break;
    case 8 :                                            //###### KEY 8 ######
    {
      if(key->trigger)    //press the key down first time;
      {
        key->trigger = 0;
        key->down_count = 0;
        // user add the one time mode code below;


      }
      else
      {
        key->down_count ++;
        if(key->down_count >= key_pressdown_time)
        { key->down_count = key_pressdown_time;
          // user add the one time mode code below;


        }
      }
    }break;
    case 9 :                                            //###### KEY 9 ######
    {
      if(key->trigger)    //press the key down first time;
      {
        key->trigger = 0;
        key->down_count = 0;
        // user add the one time mode code below;


      }
      else
      {
        key->down_count ++;
        if(key->down_count >= key_pressdown_time)
        { key->down_count = key_pressdown_time;
          // user add the one time mode code below;


        }
      }
    }break;
  }//end of /switch(key_temp.key_abs)
}

/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

