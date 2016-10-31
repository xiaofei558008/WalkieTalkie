/*This is the XiFi Communicate protocol functions.
File name :XiFi.c
All rights reserved,if the code is not authorized by STMicroelectronics.
2016-08-29 11:52:54 Tom.xiao@st.com
*/

/* Include Head.
*/
#include "XiFi.h"

Com_Protocol_TypeDef Com;

/* Protocol Init.
*/
void Com_Init(Com_Protocol_TypeDef* Temp)
{
  Temp->Com_State      = Com_STA_Test_Rec_Write_Simu;
  Temp->Host_Addr.all  = 0x0101;
  Temp->TimeOut_Count  = 0;
  Temp->Com_TIM        = TIM2;
}

/* Protocol Main Loop.
*/
void Com_Main_Loop(Com_Protocol_TypeDef* Temp)
{
  switch(Temp->Com_State)
  {
    case Com_STA_Free               :Com_Free(Temp);
    break;

    case Com_STA_Send               :Com_Send_Frame(Temp);
    break;

    case Com_STA_Receive            :Com_Receive_Management(Temp);
    break;

    case Com_STA_ACK_Wait           :Com_ACK_Wait(Temp);
    break;

    case Com_STA_Repeater           :Com_Repeater(Temp);
    break;

    //Only for Protocol Testing.
    case Com_STA_Test_Rec_Read_Simu :Com_Receive_Simulation_Read_Test(Temp);
                                     Com_Receive_Frame_IRQ(Temp);
    break;

    case Com_STA_Test_Rec_Write_Simu:Com_Receive_Simulation_Write_Test(Temp);
                                     Com_Receive_Frame_IRQ(Temp);
    break;
  }
}

/* Protocol Send:
   Host Addr:       Host Addr H      | Host Addr L      |
   Dest Addr:       Dest_Phy_Addr_H  | Dest_Phy_Addr_L  |
   Dest Reg Addr:   Dest_Reg_Addr_HH | Dest_Reg_Addr_HL | Dest_Reg_Addr_LH | Dest_Reg_Addr_LL |
   Command:         CMD              |
   Byte Len:        Len_H            | Len_L            |
   Data Send:       Datas...
   CRC16:           CRC_L            | CRC_H            |
*/
void Com_Send_Frame(Com_Protocol_TypeDef* Temp)
{
  uint16_t i;
  u16_u8u8 CRC_Data;

  //Send Host Physical Addr.
  Temp->Send_Data_Buffer[0]  = Temp->Host_Addr.high_low.high;
  Temp->Send_Data_Buffer[1]  = Temp->Host_Addr.high_low.low;

  //Send Guest Physical Addr.
  Temp->Send_Data_Buffer[2]  = Temp->Guest_Addr.high_low.high;
  Temp->Send_Data_Buffer[3]  = Temp->Guest_Addr.high_low.low;

  //Send Reg Addr.
  Temp->Send_Data_Buffer[4]  = Temp->Reg_Addr.high_low.hh;
  Temp->Send_Data_Buffer[5]  = Temp->Reg_Addr.high_low.hl;
  Temp->Send_Data_Buffer[6]  = Temp->Reg_Addr.high_low.lh;
  Temp->Send_Data_Buffer[7]  = Temp->Reg_Addr.high_low.ll;

  //Send Command.
  Temp->Send_Data_Buffer[8]  = Temp->CMD;

  //Send Data Len.
  Temp->Len.all             += Prot_Frame_Head_Length;
  Temp->Send_Data_Buffer[9]  = Temp->Len.high_low.high;
  Temp->Send_Data_Buffer[10] = Temp->Len.high_low.low;

  //Calculate CRC.
  CRC_Data.all = Com_CRC16_Calculate(Temp->Send_Data_Buffer,
                                     Temp->Len.all
                                    );

  Temp->Len.all                            += 2;
  Temp->Send_Data_Buffer[Temp->Len.all - 2] = CRC_Data.high_low.low;
  Temp->Send_Data_Buffer[Temp->Len.all - 1] = CRC_Data.high_low.high;

  // Send Byte.
  for(i = 0; i < Temp->Len.all; i ++)
  {
    //Send Datas.
    Com_Send_Byte(*(Temp->Send_Data_Buffer + i));
  }

  //Reset Len.
  Temp->Len.all -= (Prot_Frame_Head_Length + 2);

  //Set Communication State into ACK-Wait Mode.
  if(Temp->Len.all == 0)
  {
    Temp->Com_State = Com_STA_Free;
  }

  //Frame Send Finish CallBack.
  Com_Frame_Send_Finish_CallBack();
}

/* Protocol Receive Simulation Test Function.
*/
void Com_Receive_Simulation_Read_Test(Com_Protocol_TypeDef* Temp)
{
  u16_u8u8 CRC_Temp;

  Temp->Receive_Data_Buffer[0]  = 0x01;
  Temp->Receive_Data_Buffer[1]  = 0x02;                            //Host Address.

  Temp->Receive_Data_Buffer[2]  = Temp->Host_Addr.high_low.high;
  Temp->Receive_Data_Buffer[3]  = Temp->Host_Addr.high_low.low;    //Guest Address.

  Temp->Receive_Data_Buffer[4]  = 0x00;
  Temp->Receive_Data_Buffer[5]  = 0x00;
  Temp->Receive_Data_Buffer[6]  = 0x00;
  Temp->Receive_Data_Buffer[7]  = 0x00;                            //Register Address

  Temp->Receive_Data_Buffer[8]  = Com_CMD_Read;                    //Command

  Temp->Receive_Data_Buffer[9]  = 0x00;
  Temp->Receive_Data_Buffer[10] = 0x10;                            //Length.

  CRC_Temp.all = Com_CRC16_Calculate(Temp->Receive_Data_Buffer,    //CRC Calculate.
                                     11
                                    );

  Temp->Receive_Data_Buffer[11] = CRC_Temp.high_low.low;
  Temp->Receive_Data_Buffer[12] = CRC_Temp.high_low.high;

  Temp->Flag_Receive_Buffer_Vaild = true;
}


/* Protocol Receive Simulation Test Function.
*/
void Com_Receive_Simulation_Write_Test(Com_Protocol_TypeDef* Temp)
{
  u16_u8u8 CRC_Temp;

  Temp->Receive_Data_Buffer[0]  = 0x01;
  Temp->Receive_Data_Buffer[1]  = 0x02;                             //Host Address.

  Temp->Receive_Data_Buffer[2]  = Temp->Host_Addr.high_low.high;
  Temp->Receive_Data_Buffer[3]  = Temp->Host_Addr.high_low.low;     //Guest Address.

  Temp->Receive_Data_Buffer[4]  = 0x00;
  Temp->Receive_Data_Buffer[5]  = 0x00;
  Temp->Receive_Data_Buffer[6]  = 0x00;
  Temp->Receive_Data_Buffer[7]  = 0x00;                             //Register Address

  Temp->Receive_Data_Buffer[8]  = Com_CMD_Write;                    //Command

  Temp->Receive_Data_Buffer[9]  = 0x00;
  Temp->Receive_Data_Buffer[10] = 0x10;                             //Length.

  Temp->Receive_Data_Buffer[11] = 0xff;                             //Data...
  Temp->Receive_Data_Buffer[12] = 0xee;
  Temp->Receive_Data_Buffer[13] = 0xdd;
  Temp->Receive_Data_Buffer[14] = 0xcc;
  Temp->Receive_Data_Buffer[15] = 0xbb;
  Temp->Receive_Data_Buffer[16] = 0xaa;
  Temp->Receive_Data_Buffer[17] = 0x99;
  Temp->Receive_Data_Buffer[18] = 0x88;
  Temp->Receive_Data_Buffer[19] = 0x77;
  Temp->Receive_Data_Buffer[20] = 0x66;
  Temp->Receive_Data_Buffer[21] = 0x55;
  Temp->Receive_Data_Buffer[22] = 0x44;
  Temp->Receive_Data_Buffer[23] = 0x33;
  Temp->Receive_Data_Buffer[24] = 0x22;
  Temp->Receive_Data_Buffer[25] = 0x11;
  Temp->Receive_Data_Buffer[26] = 0x00;

  CRC_Temp.all = Com_CRC16_Calculate(Temp->Receive_Data_Buffer,    //CRC Calculate.
                                     27
                                    );

  Temp->Receive_Data_Buffer[27] = CRC_Temp.high_low.low;
  Temp->Receive_Data_Buffer[28] = CRC_Temp.high_low.high;

  Temp->Flag_Receive_Buffer_Vaild = true;
}

/* Protocol Receive (In Timer OverFlow Interrupt):
*/
void Com_Receive_Frame_IRQ(Com_Protocol_TypeDef* Temp)
{
  uint16_t i;

  Temp->Guest_Addr.high_low.high = Temp->Receive_Data_Buffer[2];
  Temp->Guest_Addr.high_low.low  = Temp->Receive_Data_Buffer[3];

  for(i = 0; i < Prot_Guest_Addr_Amount; i ++)
  {
    //Scan the guest list;
    if((Temp->Guest_Addr_List[i].all != 0) && (Temp->Guest_Addr_List[i].all != Temp->Guest_Addr.all))
    {
      continue;
    }

    //Not in guest list.
    else if(Temp->Guest_Addr_List[i].all == 0)
    {
      Temp->Guest_Addr_List[i].all = Temp->Guest_Addr.all;
      break;
    }

    //Already in Guest list
    else
    {
      break;
    }
  }

  Temp->Reg_Addr.high_low.hh = Temp->Receive_Data_Buffer[4];
  Temp->Reg_Addr.high_low.hl = Temp->Receive_Data_Buffer[5];
  Temp->Reg_Addr.high_low.lh = Temp->Receive_Data_Buffer[6];
  Temp->Reg_Addr.high_low.ll = Temp->Receive_Data_Buffer[7];

  Temp->CMD                  = (Com_CMD_TypeDef)Temp->Receive_Data_Buffer[8];
  Temp->Len.high_low.high    = Temp->Receive_Data_Buffer[9];
  Temp->Len.high_low.low     = Temp->Receive_Data_Buffer[10];

  Temp->Flag_Receive_Buffer_Vaild = true;
  Temp->Com_State            = Com_STA_Receive;
}

/* Protocol Receive Manage Function(Main Loop).
*/
void Com_Receive_Management(Com_Protocol_TypeDef* Temp)
{
  //Data Available.
  if(Temp->Flag_Receive_Buffer_Vaild == true)
  {
    Temp->Flag_Receive_Buffer_Vaild = false;

    //Physical Address Judge.
    if(Temp->Guest_Addr.all == Temp->Host_Addr.all)
    {
      Temp->Guest_Addr.high_low.high = Temp->Receive_Data_Buffer[0];
      Temp->Guest_Addr.high_low.low  = Temp->Receive_Data_Buffer[1];

      //Switch CMD -- CRC Calculate.
      switch(Temp->CMD)
      {
        case Com_CMD_Read: //CRC Passed.
                           if(Com_CRC16_Calculate(Temp->Receive_Data_Buffer,
                                                  13
                                                 ) == 0
                             )
                           { Com_Read_CallBack(Temp);
                           }
                           else
                           {
                             //ACK false.
                             Com_ACK_Send(Temp, false);
                           }
        break;

        case Com_CMD_Write://CRC Passed.
                           if(Com_CRC16_Calculate(Temp->Receive_Data_Buffer,
                                                  (Temp->Len.all + 13)
                                                 ) == 0
                             )
                           { Com_Write_CallBack(Temp);
                           }
                           else
                           {
                             //ACK false.
                             Com_ACK_Send(Temp, false);
                           }
        break;

        case Com_CMD_Free:
        break;

        case Com_CMD_Del:
        break;

        case Com_CMD_Encry:
        break;
      }
    }
  }
}

/* Send Frame.
*/
void Com_Send_Byte(uint8_t Data)
{
  while((UART4->ISR & USART_ISR_TC) == 0);
  UART4->TDR = Data;
}

/* Protocol Receive one Byte.
*/
uint8_t Com_Receive_Byte(void)
{
  while((UART4->ISR & USART_ISR_RXNE) == 0);
  return UART4->RDR ;
}

/* UID Encryption.
*/
void Com_UID_Encryption(uint32_t Cmd)
{
/*
  uint32_t Temp;

  Temp = Temp->UID;

  Temp->UID =
*/
}

/* CRC16 Software.
*/
uint16_t Com_CRC16_Calculate(uint8_t* reg_addr,
                             uint16_t frame_length
                            )
{ uint8_t move_count = 8;
  uint16_t CRC16_temp;

  CRC16_temp = 0xffff;                  //preset

  while(frame_length)
  {
    frame_length--;

    CRC16_temp ^= (*reg_addr);

    while(move_count)
    {
      move_count--;

    if(CRC16_temp & 0x0001)             //judge the last bit wheather it's 1
    {
      CRC16_temp >>= 1;
      CRC16_temp ^= Prot_CRC_Code;      //0xa001
    }
    else
    {
      CRC16_temp>>=1;
    }
    }//end of while(move_count--)

    reg_addr++;
    move_count=8;

  }//end of while(frame_length--)

  return CRC16_temp;
}

/* Address Mapping.
*/
static uint8_t* Com_Addr_Mapping(uint32_t Addr_Temp)
{
  uint8_t* Temp;

  switch(Addr_Temp)
  {
#if(Com_Mapping_Sec0_Enable == 1)
    /* Section 0
    */
    case (Com_Mapping_Addr_Offset_Sec0 + 0 ): Temp = Com_Mapping_Sec0_Addr0 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 1 ): Temp = Com_Mapping_Sec0_Addr1 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 2 ): Temp = Com_Mapping_Sec0_Addr2 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 3 ): Temp = Com_Mapping_Sec0_Addr3 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 4 ): Temp = Com_Mapping_Sec0_Addr4 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 5 ): Temp = Com_Mapping_Sec0_Addr5 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 6 ): Temp = Com_Mapping_Sec0_Addr6 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 7 ): Temp = Com_Mapping_Sec0_Addr7 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 8 ): Temp = Com_Mapping_Sec0_Addr8 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 9 ): Temp = Com_Mapping_Sec0_Addr9 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 10): Temp = Com_Mapping_Sec0_Addr10;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 11): Temp = Com_Mapping_Sec0_Addr11;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 12): Temp = Com_Mapping_Sec0_Addr12;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 13): Temp = Com_Mapping_Sec0_Addr13;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 14): Temp = Com_Mapping_Sec0_Addr14;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 15): Temp = Com_Mapping_Sec0_Addr15;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 16): Temp = Com_Mapping_Sec0_Addr16;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 17): Temp = Com_Mapping_Sec0_Addr17;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 18): Temp = Com_Mapping_Sec0_Addr18;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 19): Temp = Com_Mapping_Sec0_Addr19;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 20): Temp = Com_Mapping_Sec0_Addr20;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 21): Temp = Com_Mapping_Sec0_Addr21;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 22): Temp = Com_Mapping_Sec0_Addr22;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 23): Temp = Com_Mapping_Sec0_Addr23;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 24): Temp = Com_Mapping_Sec0_Addr24;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 25): Temp = Com_Mapping_Sec0_Addr25;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 26): Temp = Com_Mapping_Sec0_Addr26;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 27): Temp = Com_Mapping_Sec0_Addr27;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 28): Temp = Com_Mapping_Sec0_Addr28;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 29): Temp = Com_Mapping_Sec0_Addr29;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 30): Temp = Com_Mapping_Sec0_Addr30;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 31): Temp = Com_Mapping_Sec0_Addr31;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 32): Temp = Com_Mapping_Sec0_Addr32;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 33): Temp = Com_Mapping_Sec0_Addr33;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 34): Temp = Com_Mapping_Sec0_Addr34;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 35): Temp = Com_Mapping_Sec0_Addr35;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 36): Temp = Com_Mapping_Sec0_Addr36;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 37): Temp = Com_Mapping_Sec0_Addr37;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 38): Temp = Com_Mapping_Sec0_Addr38;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 39): Temp = Com_Mapping_Sec0_Addr39;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 40): Temp = Com_Mapping_Sec0_Addr40;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 41): Temp = Com_Mapping_Sec0_Addr41;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 42): Temp = Com_Mapping_Sec0_Addr42;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 43): Temp = Com_Mapping_Sec0_Addr43;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 44): Temp = Com_Mapping_Sec0_Addr44;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 45): Temp = Com_Mapping_Sec0_Addr45;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 46): Temp = Com_Mapping_Sec0_Addr46;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 47): Temp = Com_Mapping_Sec0_Addr47;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 48): Temp = Com_Mapping_Sec0_Addr48;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 49): Temp = Com_Mapping_Sec0_Addr49;break;
    //Other Address Canbe Added Below.
#endif

#if(Com_Mapping_Sec1_Enable == 1)
    /* Section 1
    */
    case (Com_Mapping_Addr_Offset_Sec0 + 0 ): Temp = Com_Mapping_Sec0_Addr0 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 1 ): Temp = Com_Mapping_Sec0_Addr1 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 2 ): Temp = Com_Mapping_Sec0_Addr2 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 3 ): Temp = Com_Mapping_Sec0_Addr3 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 4 ): Temp = Com_Mapping_Sec0_Addr4 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 5 ): Temp = Com_Mapping_Sec0_Addr5 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 6 ): Temp = Com_Mapping_Sec0_Addr6 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 7 ): Temp = Com_Mapping_Sec0_Addr7 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 8 ): Temp = Com_Mapping_Sec0_Addr8 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 9 ): Temp = Com_Mapping_Sec0_Addr9 ;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 10): Temp = Com_Mapping_Sec0_Addr10;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 11): Temp = Com_Mapping_Sec0_Addr11;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 12): Temp = Com_Mapping_Sec0_Addr12;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 13): Temp = Com_Mapping_Sec0_Addr13;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 14): Temp = Com_Mapping_Sec0_Addr14;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 15): Temp = Com_Mapping_Sec0_Addr15;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 16): Temp = Com_Mapping_Sec0_Addr16;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 17): Temp = Com_Mapping_Sec0_Addr17;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 18): Temp = Com_Mapping_Sec0_Addr18;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 19): Temp = Com_Mapping_Sec0_Addr19;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 20): Temp = Com_Mapping_Sec0_Addr20;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 21): Temp = Com_Mapping_Sec0_Addr21;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 22): Temp = Com_Mapping_Sec0_Addr22;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 23): Temp = Com_Mapping_Sec0_Addr23;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 24): Temp = Com_Mapping_Sec0_Addr24;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 25): Temp = Com_Mapping_Sec0_Addr25;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 26): Temp = Com_Mapping_Sec0_Addr26;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 27): Temp = Com_Mapping_Sec0_Addr27;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 28): Temp = Com_Mapping_Sec0_Addr28;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 29): Temp = Com_Mapping_Sec0_Addr29;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 30): Temp = Com_Mapping_Sec0_Addr30;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 31): Temp = Com_Mapping_Sec0_Addr31;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 32): Temp = Com_Mapping_Sec0_Addr32;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 33): Temp = Com_Mapping_Sec0_Addr33;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 34): Temp = Com_Mapping_Sec0_Addr34;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 35): Temp = Com_Mapping_Sec0_Addr35;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 36): Temp = Com_Mapping_Sec0_Addr36;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 37): Temp = Com_Mapping_Sec0_Addr37;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 38): Temp = Com_Mapping_Sec0_Addr38;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 39): Temp = Com_Mapping_Sec0_Addr39;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 40): Temp = Com_Mapping_Sec0_Addr40;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 41): Temp = Com_Mapping_Sec0_Addr41;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 42): Temp = Com_Mapping_Sec0_Addr42;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 43): Temp = Com_Mapping_Sec0_Addr43;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 44): Temp = Com_Mapping_Sec0_Addr44;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 45): Temp = Com_Mapping_Sec0_Addr45;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 46): Temp = Com_Mapping_Sec0_Addr46;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 47): Temp = Com_Mapping_Sec0_Addr47;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 48): Temp = Com_Mapping_Sec0_Addr48;break;
    case (Com_Mapping_Addr_Offset_Sec0 + 49): Temp = Com_Mapping_Sec0_Addr49;break;
    //Other Address Canbe Added Below.
#endif

#if(Com_Mapping_Sec2_Enable == 1)
    /* Section 2
    */
    case (Com_Mapping_Addr_Offset_Sec2 + 0 ): Temp = Com_Mapping_Sec2_Addr0 ;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 1 ): Temp = Com_Mapping_Sec2_Addr1 ;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 2 ): Temp = Com_Mapping_Sec2_Addr2 ;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 3 ): Temp = Com_Mapping_Sec2_Addr3 ;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 4 ): Temp = Com_Mapping_Sec2_Addr4 ;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 5 ): Temp = Com_Mapping_Sec2_Addr5 ;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 6 ): Temp = Com_Mapping_Sec2_Addr6 ;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 7 ): Temp = Com_Mapping_Sec2_Addr7 ;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 8 ): Temp = Com_Mapping_Sec2_Addr8 ;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 9 ): Temp = Com_Mapping_Sec2_Addr9 ;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 10): Temp = Com_Mapping_Sec2_Addr10;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 11): Temp = Com_Mapping_Sec2_Addr11;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 12): Temp = Com_Mapping_Sec2_Addr12;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 13): Temp = Com_Mapping_Sec2_Addr13;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 14): Temp = Com_Mapping_Sec2_Addr14;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 15): Temp = Com_Mapping_Sec2_Addr15;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 16): Temp = Com_Mapping_Sec2_Addr16;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 17): Temp = Com_Mapping_Sec2_Addr17;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 18): Temp = Com_Mapping_Sec2_Addr18;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 19): Temp = Com_Mapping_Sec2_Addr19;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 20): Temp = Com_Mapping_Sec2_Addr20;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 21): Temp = Com_Mapping_Sec2_Addr21;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 22): Temp = Com_Mapping_Sec2_Addr22;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 23): Temp = Com_Mapping_Sec2_Addr23;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 24): Temp = Com_Mapping_Sec2_Addr24;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 25): Temp = Com_Mapping_Sec2_Addr25;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 26): Temp = Com_Mapping_Sec2_Addr26;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 27): Temp = Com_Mapping_Sec2_Addr27;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 28): Temp = Com_Mapping_Sec2_Addr28;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 29): Temp = Com_Mapping_Sec2_Addr29;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 30): Temp = Com_Mapping_Sec2_Addr30;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 31): Temp = Com_Mapping_Sec2_Addr31;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 32): Temp = Com_Mapping_Sec2_Addr32;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 33): Temp = Com_Mapping_Sec2_Addr33;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 34): Temp = Com_Mapping_Sec2_Addr34;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 35): Temp = Com_Mapping_Sec2_Addr35;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 36): Temp = Com_Mapping_Sec2_Addr36;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 37): Temp = Com_Mapping_Sec2_Addr37;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 38): Temp = Com_Mapping_Sec2_Addr38;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 39): Temp = Com_Mapping_Sec2_Addr39;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 40): Temp = Com_Mapping_Sec2_Addr40;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 41): Temp = Com_Mapping_Sec2_Addr41;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 42): Temp = Com_Mapping_Sec2_Addr42;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 43): Temp = Com_Mapping_Sec2_Addr43;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 44): Temp = Com_Mapping_Sec2_Addr44;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 45): Temp = Com_Mapping_Sec2_Addr45;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 46): Temp = Com_Mapping_Sec2_Addr46;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 47): Temp = Com_Mapping_Sec2_Addr47;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 48): Temp = Com_Mapping_Sec2_Addr48;break;
    case (Com_Mapping_Addr_Offset_Sec2 + 49): Temp = Com_Mapping_Sec2_Addr49;break;
    //Other Address Canbe Added Below.
#endif

    /* Other Sections Canbe Added Below.
    */
  }//end of switch

  return Temp;
}

/*
*  CallBack Functions Below.
*/
/* Protocol ACK.
*/
void Com_ACK_Send(Com_Protocol_TypeDef* Temp,
                  bool                  Flag
                 )
{ if(Flag == true)
  {
    //ACK Ok.
    //Add Content into Send ACK.
    Temp->Flag_Send_Buffer_Vaild = false;
    Temp->Guest_Addr.all       = Temp->Guest_Addr.all;

    Temp->Reg_Addr.high_low.hh = 'O';
    Temp->Reg_Addr.high_low.hl = 'K';
    Temp->Reg_Addr.high_low.lh = '\r';
    Temp->Reg_Addr.high_low.ll = '\n';

    Temp->CMD                  = Com_CMD_ACK;
    Temp->Len.all              = 0;

    //Resend many times;
    Temp->Com_State = Com_STA_Send;
  }
  else
  {
    //ACK Bad.
    //Add Content into Send ACK.
    Temp->Flag_Send_Buffer_Vaild  = false;
    Temp->Guest_Addr.all       = 0x0000;

    Temp->Reg_Addr.high_low.hh = 'E';
    Temp->Reg_Addr.high_low.hl = 'R';
    Temp->Reg_Addr.high_low.lh = '\r';
    Temp->Reg_Addr.high_low.ll = '\n';

    Temp->CMD                  = Com_CMD_ACK;
    Temp->Len.all              = 0;

    //Resend many times;
    Temp->Com_State = Com_STA_Send;
  }
}

/* Protocol Free.
*/
void Com_Free(Com_Protocol_TypeDef* Temp)
{
  //Add Content into Send Free.
  Temp->Flag_Send_Buffer_Vaild = false;
  Temp->Guest_Addr.all    = 0x0000;
  Temp->Reg_Addr.all      = 0x00000000;
  Temp->CMD               = Com_CMD_Free;
  Temp->Len.all           = 0;

  //Resend many times;
  Temp->Com_State = Com_STA_Send;
}

/* ACK or ACK TimeOut.
*/
void Com_ACK_Wait(Com_Protocol_TypeDef* Temp)
{
  if(Temp->TimeOut_Count < Prot_ACK_TimeOut_Amount)
  {
    Temp->TimeOut_Count ++;

    //Resend many times;
    Temp->Com_State = Com_STA_Send;
  }
  else
  {
    //Set Timeout Flag = 1;
    Temp->Flag_Error.flag.bit0 = 1;

    //Set Protocol State Free;
    Temp->Com_State = Com_STA_Free;

    //Clear Time Out Counter.
    Temp->TimeOut_Count = 0;
  }
}

/* Repeater Function.
*/
void Com_Repeater(Com_Protocol_TypeDef* Temp)
{

}

/* Send Frame CallBack.
*/
void Com_Frame_Send_Finish_CallBack(void)
{

}

/* Receive Finish CallBack.
*/
void Com_Frame_Receive_Finish_CallBack(void)
{

}

/* Protocol Read Function.
*/
static void Com_Read_CallBack(Com_Protocol_TypeDef* Temp)
{
  uint16_t i;
  uint16_t Len_Temp = Temp->Len.all + Prot_Frame_Head_Length;

  for(i = Prot_Frame_Head_Length; i < Len_Temp; i ++)
  {
    Temp->Send_Data_Buffer[i] = *(Com_Addr_Mapping(Temp->Reg_Addr.all + i - Prot_Frame_Head_Length));
  }

  /* Setup Send Frame.
  */
  //Set Protocol State Send.
  Temp->Com_State                 = Com_STA_Send;
  Temp->Flag_Receive_Buffer_Vaild = false;
}

/* Protocol Write Function.
*/
static void Com_Write_CallBack(Com_Protocol_TypeDef* Temp)
{
  uint16_t i;
  uint16_t Len_Temp = Temp->Len.all;

  for(i = 0; i < Len_Temp; i ++)
  {
    *(Com_Addr_Mapping(Temp->Reg_Addr.all + i)) = Temp->Receive_Data_Buffer[i + Prot_Frame_Head_Length];
  }

  /* Send Ack Frame.
  */
  Com_ACK_Send(Temp,
               true
              );
}

#if 0
/* Timer Period Interrupt(3.5 Bytes Period).
*/
extern TIM_HandleTypeDef htim6;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim6)
  {
    if(Com.Flag_Receive_Buffer_Vaild == false)
    {
      //Com.Flag_Buffer_Vaild = true;

      //Com_Receive_Frame_IRQ(&Com);
    }
  }
}
#endif

/********** End of file *********** Copy Right Reserved by STMicroelectronics ***********/

