/**
******************************************************************************
* @file    
* @author  
* @version 
* @date    
* @brief   
******************************************************************************
*
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "hsAT24_EEPROM.h"
#include "hs_Common_Utill.h"

//#include "Pump_Ctrl_v2xx_Config.h"
#include "hsBSP_MCU_Config.h" 


/* Private variables ---------------------------------------------------------*/

const uint32_t dwFpAT24xx_delay_ms = (uint32_t)dwHsSW_Delay_ms;

//
int32_t (*I2C_Mem_Write)(uint16_t wDevAddr, uint16_t wMemAddr, uint16_t wMemAddrSize, uint8_t *p_uData, uint16_t wSize, uint32_t dwTimeout) = BSP_AT24_I2C_Write;
int32_t (*I2C_Mem_Read)(uint16_t wDevAddr, uint16_t wMemAddr, uint16_t wMemAddrSize, uint8_t *p_uData, uint16_t wSize, uint32_t dwTimeout) = BSP_AT24_I2C_Read;



/* Private function prototypes -----------------------------------------------*/



/* Private function  ---------------------------------------------------------*/


/**

typedef struct
{
  uint16_t wPageSize;
  uint16_t wDev_Address;
  uint16_t wMemStart_Adrress;
  uint16_t wMemAddress_Size;
  uint16_t wMemMaxSize;
}hsAT24xx_ConfigType;

* @brief :
* @param :
* @retval:
*/



/**
* @brief :
* @param :
* @retval:
*/




/* Exported functions ------------------------------------------------------- */
/**
* @brief :
* @param :
* @retval:
*/
void hsAT24xx_Config(hsAT24xx_ConfigType *p_tMemConfig, uint16_t wPageSize, uint16_t wDev_Address, uint16_t wMemStart_Adrress, uint16_t wMemAddress_Size, uint16_t wMemMaxSize)
{
  p_tMemConfig->wPageSize               = wPageSize;
  p_tMemConfig->wDev_Address            = wDev_Address;
  p_tMemConfig->wMemStart_Adrress       = wMemStart_Adrress;
  p_tMemConfig->wMemAddress_Size        = wMemAddress_Size;
  p_tMemConfig->wMemMaxSize             = wMemMaxSize;  
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t hsAT24xx_Write(hsAT24xx_ConfigType *p_tMemConfig, uint8_t *p_uData, uint16_t wStartAddress, uint32_t dwWriteDataSize)
{
  uint32_t dwMaxMeCnt = 0;
  int32_t dwCheck = 0;
  uint16_t wMemAddress = 0;
  uint16_t page_cnt=0;
    
  wMemAddress = p_tMemConfig->wMemStart_Adrress + wStartAddress;
  dwMaxMeCnt = dwWriteDataSize;
  
  if( (wMemAddress + dwWriteDataSize) >= (p_tMemConfig->wMemStart_Adrress + p_tMemConfig->wMemMaxSize) ) return 1;
   
  if(dwWriteDataSize <= p_tMemConfig->wPageSize)
  {
    dwCheck = I2C_Mem_Write(p_tMemConfig->wDev_Address, wMemAddress, p_tMemConfig->wMemAddress_Size, p_uData, dwWriteDataSize, 1000);
    if(dwCheck > 0 ) return 1;

    ((int32_t(*)(uint32_t))dwFpAT24xx_delay_ms)(10);
  }
  else
  {
    while(1)
    {
      if(dwMaxMeCnt >= p_tMemConfig->wPageSize)
      {
        dwCheck = I2C_Mem_Write(p_tMemConfig->wDev_Address, wMemAddress, p_tMemConfig->wMemAddress_Size, p_uData+(p_tMemConfig->wPageSize*page_cnt), p_tMemConfig->wPageSize, 1000);
        if(dwCheck > 0 ) return 1;
        
        ((int32_t(*)(uint32_t))dwFpAT24xx_delay_ms)(10);
        
        wMemAddress = wMemAddress + p_tMemConfig->wPageSize;
        dwMaxMeCnt = dwMaxMeCnt - p_tMemConfig->wPageSize;
        page_cnt++;
      }
      else
      {
        dwCheck = I2C_Mem_Write(p_tMemConfig->wDev_Address, wMemAddress, p_tMemConfig->wMemAddress_Size, p_uData+(p_tMemConfig->wPageSize*page_cnt), dwMaxMeCnt, 1000);
        if(dwCheck > 0 ) return 1;
        
        ((int32_t(*)(uint32_t))dwFpAT24xx_delay_ms)(10);
        
        break;
      }
    }
  }
  
  return dwCheck;
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t hsAT24xx_Read(hsAT24xx_ConfigType *p_tMemConfig, uint8_t *p_uData, uint16_t wStartAddress, uint32_t dwReadDataSize)
{
  int32_t dwCheck = 0;
  uint16_t wMemAddress = 0;
  
  wMemAddress = p_tMemConfig->wMemStart_Adrress + wStartAddress;
  if( (wMemAddress + dwReadDataSize) >= (p_tMemConfig->wMemStart_Adrress + p_tMemConfig->wMemMaxSize) ) return 1;
  
  dwCheck = I2C_Mem_Read(p_tMemConfig->wDev_Address, wMemAddress, p_tMemConfig->wMemAddress_Size, p_uData, dwReadDataSize, 1000);
  
  if(dwCheck > 0) return 1;
  
  return 0;
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
