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
#include "KLight_Project_OnOff_Drive.h"
#include "hs_Common_Utill.h"

//#include "Pump_Ctrl_v2xx_Config.h"
#include "hsBSP_MCU_Config.h" 


/* Private variables ---------------------------------------------------------*/




//-----------------------------------------------------------------------------
//BSP Aline
const uint32_t g_dwFpKLightETC_OnOff          = (uint32_t)BSP_ETC_OnOff_Device;       //void BSP_ETC_OnOff_Device(uint32_t dwDev_num, uint32_t dwPinState);
const uint32_t g_dwFpKLightETC_Input          = (uint32_t)BSP_ETC_Input_Check;         //int32_t BSP_ETC_Input_Check(uint32_t dwDev_num);

/* macro ----------------------------------------------------------------------*/
#define mcrFP_KLIGHT_ETC_ONOFF                  ((void(*)(uint32_t,uint32_t))g_dwFpKLightETC_OnOff)
#define mcrFP_KLIGHT_ETC_INPUT                  ((int32_t(*)(uint32_t))g_dwFpKLightETC_Input)


/* Private function prototypes -----------------------------------------------*/



/* Private function  ---------------------------------------------------------*/


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
void KLight_BathPower_OnOff(uint32_t dwOn_nOff)
{
  mcrFP_KLIGHT_ETC_ONOFF(4, dwOn_nOff);
}
/**
* @brief :
* @param :
* @retval:
*/
int32_t KLight_Bath_Detect_Pin1(void)
{
  int32_t dwCheck = 0;
  int32_t dwReturnValue = 0;
  
  dwCheck = mcrFP_KLIGHT_ETC_INPUT(1);
  if(dwCheck == 0) dwReturnValue = 1;
  else dwReturnValue = 0;
  
  return dwReturnValue;
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t KLight_Bath_Detect_Pin2(void)
{
  int32_t dwCheck = 0;
  int32_t dwReturnValue = 0;
  
  dwCheck = mcrFP_KLIGHT_ETC_INPUT(2);
  if(dwCheck == 0) dwReturnValue = 1;
  else dwReturnValue = 0;
  
  return dwReturnValue;
}

/**
* @brief :
* @param :
* @retval:
mcrFP_KLIGHT_ETC_ONOFF
*/
void KLight_Buzzer_OnOff(uint32_t dwDuration_ms, uint32_t dwRePlayCnt)
{
  uint32_t dwCnt;
  
  for(dwCnt = 0; dwCnt < dwRePlayCnt; dwCnt++)
  {
    mcrFP_KLIGHT_ETC_ONOFF(0, 1);
    BSP_SystemHW_Delay_msec(dwDuration_ms);
    mcrFP_KLIGHT_ETC_ONOFF(0, 0);
    BSP_SystemHW_Delay_msec(dwDuration_ms);
  }
}

/**
* @brief :
* @param :
* @retval:
Pin High Fan Off, Pin Low Fan On
*/
void KLight_Fan_OnOff(uint32_t dwOn_nOff)
{
  uint32_t dwMod;
  
  if(dwOn_nOff ==1) dwMod = 0;
  else dwMod = 1;
  
  mcrFP_KLIGHT_ETC_ONOFF(1, dwMod);
  
}

/**
* @brief :
* @param :
* @retval:
*/
void KLight_Heater_OnOff(uint32_t dwOn_nOff)
{
  uint32_t dwMod;
  
  if(dwOn_nOff ==1) dwMod = 1;
  else dwMod = 0;
  
  mcrFP_KLIGHT_ETC_ONOFF(4, dwMod);
}


/**
* @brief :
* @param :
* @retval:
*/


/**
* @brief :
* @param :
* @retval:
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
