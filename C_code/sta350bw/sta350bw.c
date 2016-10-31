/**
******************************************************************************
* @file    STA350BW.h
* @author  Central Labs
* @version V1.0.0
* @date    18-August-2015
* @brief   This file provides the STA350BW SOUND TERMINAL audio driver.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "sta350bw.h"

/** @defgroup STA350BW_Private_Variables
* @{
*/
SOUNDTERMINAL_Object_t STA350BW_I2C_Drv =
{
  .Instance      = 0x00,
  .DevAddr       = STA350BW_ADDRESS_1,
  .IO_Init       = STA350BW_GPIO_I2S_Init,
  .IO_Read       = STA350BW_I2C_Read,
  .IO_Write      = STA350BW_I2C_Write,
  .IO_ReadMulti  = STA350BW_I2C_ReadMulti,
  .IO_WriteMulti = STA350BW_I2C_WriteMulti,
  .IO_Delay      = STA350BW_I2C_Delay
};

/* Audio codec driver structure initialization */
SOUNDTERMINAL_Drv_t STA350BW_Drv =
{
  .Init         = STA350BW_Init,
  .DeInit       = STA350BW_DeInit,
  .ReadID       = STA350BW_ReadID,
  .Play         = STA350BW_Play,
  .Pause        = STA350BW_Pause,
  .Resume       = STA350BW_Resume,
  .Stop         = STA350BW_Stop,
  .PowerOn      = STA350BW_PowerOn,
  .PowerOff     = STA350BW_PowerOff,
  .Reset        = STA350BW_Reset,
  .SetEq        = STA350BW_SetEq,
  .SetTone      = STA350BW_SetTone,
  .SetMute      = STA350BW_SetMute,
  .SetVolume    = STA350BW_SetVolume,
  .SetFrequency = STA350BW_SetFrequency,
  .SetDSPOption = STA350BW_SetDSPOption
};

/**
* @}
*/

/** @defgroup STA350BW_Private_FunctionPrototypes
* @{
*/
static int32_t writeRAMSet(SOUNDTERMINAL_Object_t* pObj,
                           uint8_t                 RAM_block,
                           uint8_t                 RAM_address,
                           uint8_t*                pIn
                          );
/**
* @}
*/

/** @defgroup STA350BW_Private_Functions
* @{
*/

/**
* @brief        Initializes the STA350BW and the control interface.
* @param        pObj: object related to the current device instance.
* @param        volume: master volume to be setup.
* @param        samplingFreq: sampling frequency.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_Init(SOUNDTERMINAL_Object_t* pObj,
                      uint16_t                volume,
                      uint32_t                samplingFreq,
                      void*                   p
                     )
{ uint8_t tmp = 0x00;

  /* Low level I2C init */
  pObj->IO_Init();

  /* Set Master clock depending on sampling frequency*/
  if(STA350BW_SetFrequency(pObj, samplingFreq, 0) != 0)
  {
    return STA350BW_ERROR;
  }

  pObj->IO_Delay(500);

  /*Read Status Register*/
  if(pObj->IO_Read(pObj->DevAddr, STA350BW_STATUS, &tmp) != 0)
  {
    return STA350BW_ERROR;
  }

  if(tmp != 0x7F)
  {
    /*Status register highlights undesired behaviour
    (PLL not locked, ...)*/
    return STA350BW_ERROR;
  }

  /*Setup Master volume */
  if(STA350BW_SetVolume(pObj, STA350BW_CHANNEL_MASTER, volume, 0) != 0)
  {
    return STA350BW_ERROR;
  }

  if(pObj->IO_Read(pObj->DevAddr, STA350BW_CONF_REGF, &tmp) != 0)
  {
    return STA350BW_ERROR;
  }

  tmp &= ~0x80;
  tmp |= 0x80;

  /*Enable Power Out Stage*/
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_CONF_REGF, tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Deinitializes the STA350BW and the control interface.
* @param        pObj: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_DeInit(SOUNDTERMINAL_Object_t* pObj,
                        void*                   p
                       )
{
  if(STA350BW_PowerOff(pObj, 0) != 0)
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Read the device ID.
* @param        pObj: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_ReadID(SOUNDTERMINAL_Object_t* pObj,
                        void*                   p
                       )
{
  return STA350BW_OK;
}

/**
* @brief        Start the audio play.
* @param        pObj: object related to the current device instance.
* @param        *pData: pointer to audio data.
* @param        *p: pointer to optional additional functions.
* @param        Size: size of the data buffer.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_Play(SOUNDTERMINAL_Object_t* pObj,
                      uint16_t*               pData,
                      uint16_t                Size,
                      void*                   p
                     )
{ return STA350BW_OK;
}

/**
* @brief        Pause the audio play.
* @param        pObj: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_Pause(SOUNDTERMINAL_Object_t* pObj,
                       void*                   p
                      )
{ /* Mute the output*/
  if(STA350BW_SetMute(pObj, STA350BW_CHANNEL_MASTER, STA350BW_ENABLE, 0)
      != 0)
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Resume the audio play.
* @param        pObj: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_Resume(SOUNDTERMINAL_Object_t* pObj, void* p)
{
  /* Unmute the output*/
  if(STA350BW_SetMute(pObj,
                      STA350BW_CHANNEL_MASTER,
                      STA350BW_DISABLE,
                      0
                     ) != 0
    )
  {return STA350BW_ERROR;
  }

  return STA350BW_OK;
}

/**
* @brief        Control the mute features of the STA350BW.
* @param        pObj: object related to the current device instance.
* @param        channel: channel to be muted.
*             This parameter can be a value of @ref STA350BW_channel_define
* @param        state: eable disable parameter
*             This parameter can be a value of @ref STA350BW_state_define
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_SetMute(SOUNDTERMINAL_Object_t* pObj,
                         uint8_t channel,
                         uint8_t state,
                         void* p
                        )
{ uint8_t tmp;

  if(pObj->IO_Read(pObj->DevAddr, STA350BW_MUTE, &tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  if(state == STA350BW_ENABLE)
  {
    tmp |= channel;
  } else
  {
    tmp &= ~channel;
  }

  if(pObj->IO_Write(pObj->DevAddr, STA350BW_MUTE, tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Control the volume features of the STA350BW.
* @param        pObj: object related to the current device instance.
* @param        channel: channel to be controlled.
*             This parameter can be a value of @ref STA350BW_channel_define
* @param        volume: volume to be set
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_SetVolume(SOUNDTERMINAL_Object_t* pObj,
                           uint8_t                 channel,
                           uint8_t                 value,
                           void*                   p
                          )
{ /*Setup volume */
  uint8_t tmp = value;
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_MVOL + channel, tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        set the sampling frequency for STA350BW.
* @param        pObj: object related to the current device instance.
* @param        AudioFreq: audio frequency to be set
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_SetFrequency(SOUNDTERMINAL_Object_t* pObj,
                              uint32_t                AudioFreq,
                              void*                   p
                             )
{ uint8_t tmp;

  if(pObj->IO_Read(pObj->DevAddr, STA350BW_CONF_REGA, &tmp) != 0)
  {
    return STA350BW_ERROR;
  }

  tmp &= ~0x1F;

  if(AudioFreq == STA350BW_Fs_32000 || AudioFreq == STA350BW_Fs_44100
      || AudioFreq == STA350BW_Fs_48000)
  {
    tmp |= STA350BW_MCLK_256_LR_48K;
  }
  else if(AudioFreq == STA350BW_Fs_88200 || AudioFreq == STA350BW_Fs_96000)
  {
    tmp |= STA350BW_MCLK_256_LR_96K;
  } else {
    return STA350BW_ERROR;
  }

  if(pObj->IO_Write(pObj->DevAddr, STA350BW_CONF_REGA, tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Set equalization parameters for STA350BW biquad section.
* @param        pObj: object related to the current device instance.
* @param        ramBlock: ram block to be set
* @param        filterNumber: filter number
* @param        *filterValues: pointer to a uint32_t array containing filter coefficients
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
uint8_t filterValuesOut[20];
int32_t STA350BW_SetEq(SOUNDTERMINAL_Object_t* pObj,
                       uint8_t                 ramBlock,
                       uint8_t                 filterNumber,
                       uint32_t*               filterValues,
                       void*                   p
                      )
{ /*5 is due to the ram adressing: first filter is on the adresses 0x00 to 0x04; the second is on 0x05 to 0x09 ...*/
  writeRAMSet(pObj,
              ramBlock,
              filterNumber * 5,
              (uint8_t*) filterValues
             );

  return STA350BW_OK;
}

/**
* @brief        Set tone value in the STA350BW tone register.
* @param        pObj: object related to the current device instance.
* @param        toneGain: gain of the tone control
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_SetTone(SOUNDTERMINAL_Object_t* pObj,
                         uint8_t toneGain,
                         void* p
                        )
{ uint8_t tmp = toneGain;

  if(pObj->IO_Write(pObj->DevAddr, STA350BW_TONE, tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Power on the device.
* @param        pObj: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_PowerOn(SOUNDTERMINAL_Object_t* pObj, void* p)
{
  uint8_t tmp;
  if(pObj->IO_Read(pObj->DevAddr, STA350BW_CONF_REGF, &tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  tmp |= 0xC0;
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_CONF_REGF, tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Power off the device.
* @param        pObj: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_PowerOff(SOUNDTERMINAL_Object_t* pObj, void* p)
{
  uint8_t tmp;
  if(pObj->IO_Read(pObj->DevAddr, STA350BW_CONF_REGF, &tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  tmp &= ~0xC0;
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_CONF_REGF, tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief        Stop audio stream.
* @param        pObj: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_Stop(SOUNDTERMINAL_Object_t* pObj, void* p)
{
  return STA350BW_OK;
}

/**
* @brief        Reset device.
* @param        pObj: object related to the current device instance.
* @param        *p: pointer to optional additional functions.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t STA350BW_Reset(SOUNDTERMINAL_Object_t* pObj, void* p)
{
  return STA350BW_OK;
}

/**
* @brief  This function can be used to set advanced DSP options in order to
*       use advanced features on the STA350BW device.
* @param  pObj: object related to the current device instance.
* @param  option: specific option to be setted up
*       This parameter can be a value of @ref STA350BW_DSP_option_selection
* @param  state: state of the option to be controlled. Depending on the selected
*       DSP feature to be controlled, this value can be either ENABLE/DISABLE
*       or a specific numerical parameter related to the specific DSP function.
*       This parameter can be a value of @ref STA350BW_state_define
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
int32_t  STA350BW_SetDSPOption(SOUNDTERMINAL_Object_t* pObj,
                               uint8_t                 option,
                               uint8_t                 state,
                               void*                   p
                              )
{ uint8_t tmp = 0;

  switch (option)
  {
    case STA350BW_DSPB:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CONF_REGD, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x04;
        tmp |= state << 0x02;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CONF_REGD, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_HPB:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CONF_REGD, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x01;
        tmp |= state << 0x00;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CONF_REGD, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_DEMP:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CONF_REGD, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x02;
        tmp |= state << 0x01;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CONF_REGD, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_BQL:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CONF_REGD, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x08;
        tmp |= state << 0x04;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CONF_REGD, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_BQ5:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CONFX, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x04;
        tmp |= state << 0x02;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CONFX, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_BQ6:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CONFX, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x02;
        tmp |= state << 0x01;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CONFX, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_BQ7:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CONFX, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x01;
        tmp |= state << 0x00;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CONFX, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_C1EQBP:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_C1CFG, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x02;
        tmp |= state << 0x01;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_C1CFG, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_C2EQBP:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_C2CFG, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x02;
        tmp |= state << 0x01;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_C2CFG, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_C1TCB:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_C1CFG, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x01;
        tmp |= state << 0x00;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_C1CFG, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_C2TCB:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_C2CFG, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x01;
        tmp |= state << 0x00;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_C2CFG, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_C1VBP:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_C1CFG, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x04;
        tmp |= state << 0x02;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_C1CFG, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_C2VBP:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_C2CFG, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x04;
        tmp |= state << 0x02;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_C2CFG, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_EXT_RANGE_BQ1:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CXT_B4B1, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x03;
        tmp |= (state>>1);

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CXT_B4B1, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_EXT_RANGE_BQ2:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CXT_B4B1, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x0C;
        tmp |= (state>>1) << 2;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CXT_B4B1, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_EXT_RANGE_BQ3:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CXT_B4B1, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x30;
        tmp |= (state>>1) << 4;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CXT_B4B1, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_EXT_RANGE_BQ4:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CXT_B4B1, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0xC0;
        tmp |= (state>>1) << 6;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CXT_B4B1, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_EXT_RANGE_BQ5:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CXT_B7B5, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x03;
        tmp |= (state>>1);

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CXT_B7B5, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_EXT_RANGE_BQ6:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CXT_B7B5, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x0C;
        tmp |= (state>>1) << 2;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CXT_B7B5, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_EXT_RANGE_BQ7:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_CXT_B7B5, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x30;
        tmp |= (state>>1) << 4;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_CXT_B7B5, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
    case STA350BW_RAM_BANK_SELECT:
      {
        if(pObj->IO_Read(pObj->DevAddr, STA350BW_EQCFG, &tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        tmp &= ~0x03;
        tmp |= state;

        if(pObj->IO_Write(pObj->DevAddr, STA350BW_EQCFG, tmp) != 0)
        {
          return STA350BW_ERROR;
        }
        break;
      }
  }
  return STA350BW_OK;
}

/*
*  Read All Register.
*/
void STA350BW_Read_All_Reg(uint8_t* Buffer_Temp, uint8_t Len)
{
  STA350BW_I2C_ReadMulti(Buffer_Temp, STA350BW_ADDRESS_1, 0x00, Len);
}

/**
* @brief        private function for writing a RAM set.
* @param        pObj: object related to the current device instance.
* @param        RAM_block: ram block to be written.
* @param        RAM_address: ram address to be written.
* @param        *pIn: pointer to the desired value to be write.
* @retval       STA350BW_OK if correct setup, STA350BW_ERROR otherwise
*/
static int32_t writeRAMSet(SOUNDTERMINAL_Object_t* pObj,
                           uint8_t                 RAM_block,
                           uint8_t                 RAM_address,
                           uint8_t*                pIn
                          )
{ uint8_t tmp = 0x00;

  /*choose block*/
  if(pObj->IO_Read(pObj->DevAddr, STA350BW_EQCFG, &tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  tmp &= ~0x03;
  RAM_block &= 0x03;
  tmp |= RAM_block;
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_EQCFG, tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  /*set address*/
  if(pObj->IO_Read(pObj->DevAddr, STA350BW_CFADDR, &tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  tmp &= ~0x3F;
  RAM_address &= 0x3F;
  tmp |= RAM_address;
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_CFADDR, tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  /*write*/
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_B1CF1, pIn[2]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_B1CF2, pIn[1]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_B1CF3, pIn[0]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_B2CF1, pIn[6]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_B2CF2, pIn[5]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_B2CF3, pIn[4]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_A1CF1, pIn[10]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_A1CF2, pIn[9]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_A1CF3, pIn[8]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_A2CF1, pIn[14]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_A2CF2, pIn[13]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_A2CF3, pIn[12]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_B0CF1, pIn[18]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_B0CF2, pIn[17]) != 0)
  {
    return STA350BW_ERROR;
  }
  if(pObj->IO_Write(pObj->DevAddr, STA350BW_B0CF3, pIn[16]) != 0)
  {
    return STA350BW_ERROR;
  }
  /*Set WA PIN*/
  if(pObj->IO_Read(pObj->DevAddr, STA350BW_CFUD, &tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  tmp &= ~0x02;
  tmp = 0x02;

  if(pObj->IO_Write(pObj->DevAddr, STA350BW_CFUD, tmp) != 0)
  {
    return STA350BW_ERROR;
  }
  return STA350BW_OK;
}

/**
* @brief  Configures STA350BW I2C interface.
* @param  None
* @retval HAL_OK if everithing went fine, HAL_ERROR otherwise
*/
uint8_t STA350BW_GPIO_I2S_Init(void)
{
  //IIS Configure.
  //Clock Enable.
  __HAL_RCC_SPI2_CLK_ENABLE();

#if 0
  //Config I2S.
  SPI2->CR2 = 0x02;
  SPI2->I2SCFGR = 0x0a00;
  SPI2->I2SPR = 0x0202;

  //I2S2 Enable.
  SPI2->I2SCFGR |= 0x0400;
#endif

  //PWRDN.
  HAL_GPIO_WritePin(STA350BW_PWRDN_PORT,STA350BW_PWRDN_PIN,GPIO_PIN_RESET);
  STA350BW_I2C_Delay(50);
  HAL_GPIO_WritePin(STA350BW_PWRDN_PORT,STA350BW_PWRDN_PIN,GPIO_PIN_SET);

  //RESET.
  HAL_GPIO_WritePin(STA350BW_RESET_PORT,STA350BW_RESET_PIN,GPIO_PIN_RESET);
  STA350BW_I2C_Delay(50);
  HAL_GPIO_WritePin(STA350BW_RESET_PORT,STA350BW_RESET_PIN,GPIO_PIN_SET);

  return HAL_OK;
}

/**
* @brief  Read multiple bytes using I2C interface.
* @param  *pBuffer: pointer to the array that will contain read data
* @param  addr: device address
* @param  reg: register number
* @param  length: length of the data to be read
* @retval HAL_OK if everithing went fine, HAL_ERROR otherwise
*/
uint8_t STA350BW_I2C_ReadMulti(uint8_t *pBuffer, uint8_t addr, uint8_t reg, uint16_t length)
{
  if(HAL_I2C_Mem_Read(&STA350BW_IICx,
                      addr,
                      (uint16_t)reg,
                      I2C_MEMADD_SIZE_8BIT,
                      pBuffer,
                      length,
                      STA350BW_IIC_TIMOUT
                     ) != HAL_OK
    )
  { return HAL_ERROR;
  }
  return HAL_OK;
}

/**
* @brief  Write multiple bytes using I2C interface.
* @param  *pBuffer: pointer to the array that contains data to be written
* @param  addr: device address
* @param  reg: register number
* @param  length: length of the data to be written
* @retval HAL_OK if everithing went fine, HAL_ERROR otherwise
*/
uint8_t STA350BW_I2C_WriteMulti(uint8_t* pBuffer, uint8_t addr, uint8_t reg, uint16_t length)
{
  if(HAL_I2C_Mem_Write(&STA350BW_IICx,
                       addr,
                       (uint16_t)reg,
                       I2C_MEMADD_SIZE_8BIT,
                       pBuffer,
                       length,
                       STA350BW_IIC_TIMOUT
                      ) != HAL_OK
    )
  { return HAL_ERROR;
  }
  return HAL_OK;
}

/**
* @brief  Read a single byte using I2C interface.
* @param  addr: device address
* @param  reg: register number
* @param  *value: pointer that will contain read byte
* @retval HAL_OK if everithing went fine, HAL_ERROR otherwise
*/
uint8_t STA350BW_I2C_Read(uint8_t addr, uint8_t reg, uint8_t* value)
{
  if(HAL_I2C_Mem_Read(&STA350BW_IICx,
                       addr,
                       (uint16_t)reg,
                       I2C_MEMADD_SIZE_8BIT,
                       value,
                       1,
                       STA350BW_IIC_TIMOUT
                     ) != HAL_OK
    )
  { return HAL_ERROR;
  }
  return HAL_OK;
}

/**
* @brief  Write a single byte using I2C interface.
* @param  addr: device address
* @param  reg: register number
* @param  value: value to be written
* @retval HAL_OK if everithing went fine, HAL_ERROR otherwise
*/
uint8_t STA350BW_I2C_Write(uint8_t addr, uint8_t reg, uint8_t value)
{
  uint8_t Temp = value;

  if(HAL_I2C_Mem_Write(&STA350BW_IICx,
                       addr,
                       (uint16_t)reg,
                       I2C_MEMADD_SIZE_8BIT,
                       &Temp,
                       1,
                       STA350BW_IIC_TIMOUT
                      ) != HAL_OK
    )
  { return HAL_ERROR;
  }
  return HAL_OK;
}

/**
* @brief  Delay function that will be usedinside the componens driver.
* @param  delay_ms: amount of delay, in ms
* @retval HAL_OK
*/
uint8_t STA350BW_I2C_Delay(uint32_t delay_ms)
{
  HAL_Delay(delay_ms);
  return HAL_OK;
}

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


