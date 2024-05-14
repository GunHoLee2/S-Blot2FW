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
#include "main.h"
#include "hsTB6617_PeristalticPumpDrive.h"
#include "hs_Common_Utill.h"

//#include "Pump_Ctrl_v2xx_Config.h"
#include "hsBSP_MCU_Config.h" 


#define TB6617_DEVICE_MAXCNT            10


/* Private variables ---------------------------------------------------------*/
uint32_t g_dwPumpRunningTime[TB6617_DEVICE_MAXCNT];
uint32_t g_dwPumpGetSystickSet[TB6617_DEVICE_MAXCNT];
uint32_t g_dwPumpTimCheck[TB6617_DEVICE_MAXCNT];

//-----------------------------------------------------------------------------
//BSP Aline
const uint32_t g_dwFpTB6617_ENABLE      = (uint32_t)BSP_TB6617_Enable;            //void BSP_TB6617_Enable(uint32_t dwDev_num);
const uint32_t g_dwFpTB6617_DISABLE     = (uint32_t)BSP_TB6617_Diasble;           //void BSP_TB6617_Diasble(uint32_t dwDev_num);
const uint32_t g_dwFpTB6617_BREAK       = (uint32_t)BSP_TB6617_Break;             //void BSP_TB6617_Break(uint32_t dwDev_num);
const uint32_t g_dwFpTB6617_STOP        = (uint32_t)BSP_TB6617_Stop;              //void BSP_TB6617_Stop(uint32_t dwDev_num);
const uint32_t g_dwFpTB6617_CW          = (uint32_t)BSP_TB6617_CW;                //void BSP_TB6617_CW(uint32_t dwDev_num);
const uint32_t g_dwFpTB6617_CCW         = (uint32_t)BSP_TB6617_CCW;               //void BSP_TB6617_CCW(uint32_t dwDev_num);


const uint32_t g_dwFpPump_Delay_msec   = (uint32_t)BSP_SystemHW_Delay_msec;
const uint32_t g_dwFpPumpUseSystickGet = (uint32_t)BSP_SystickTimCountGet;       //uint32_t BSP_SystickTimCountGet(void)

/* macro ----------------------------------------------------------------------*/
#define mcrFP_TB6617_ENABLE             ((void(*)(uint32_t))g_dwFpTB6617_ENABLE)
#define mcrFP_TB6617_DISABLE            ((void(*)(uint32_t))g_dwFpTB6617_DISABLE)
#define mcrFP_TB6617_BREAK              ((void(*)(uint32_t))g_dwFpTB6617_BREAK)
#define mcrFP_TB6617_STOP               ((void(*)(uint32_t))g_dwFpTB6617_STOP)
#define mcrFP_TB6617_CW                 ((void(*)(uint32_t))g_dwFpTB6617_CW)
#define mcrFP_TB6617_CCW                ((void(*)(uint32_t))g_dwFpTB6617_CCW)

#define mcrFP_PUMP_DELAY_MS             ((void(*)(uint32_t))g_dwFpPump_Delay_msec)
#define mcrFP_PUMP_SYSTICK_GET          ((uint32_t(*)(void))g_dwFpPumpUseSystickGet)

/* Private function prototypes -----------------------------------------------*/



/* Private function  ---------------------------------------------------------*/
/**
* @brief :
* @param :
* @retval:
*/
void Pump_SystickTimerSet(uint32_t TimerBank_num)
{
  if(TimerBank_num >= TB6617_DEVICE_MAXCNT) return;
  
  g_dwPumpGetSystickSet[TimerBank_num] = mcrFP_PUMP_SYSTICK_GET();
  g_dwPumpTimCheck[TimerBank_num] = 1;
}

/**
* @brief :
* @param :
* @retval:
*/
uint32_t Pump_SystickTimerCurrentValue(uint32_t TimerBank_num)
{
 uint32_t dwReturnValue;
 uint32_t dwShotTime;
 
 if(TimerBank_num >= TB6617_DEVICE_MAXCNT) return 0;
 
 dwShotTime = mcrFP_PUMP_SYSTICK_GET();
 
 if(g_dwPumpGetSystickSet[TimerBank_num] > dwShotTime)
 {
   dwReturnValue = (0xFFFFFFFF - g_dwPumpGetSystickSet[TimerBank_num]) + dwShotTime;
 }
 else
 {
   dwReturnValue = dwShotTime - g_dwPumpGetSystickSet[TimerBank_num];
 }
 
 return dwReturnValue;
}

/**
* @brief :
* @param :
* @retval:
*/
void Pump_RunningTimeAdder(uint32_t dwDev_num)
{
  if(dwDev_num >= TB6617_DEVICE_MAXCNT) return;
  
  if(g_dwPumpTimCheck[dwDev_num] == 1)
  {
    g_dwPumpRunningTime[dwDev_num] = g_dwPumpRunningTime[dwDev_num] + Pump_SystickTimerCurrentValue(dwDev_num);
    
    g_dwPumpTimCheck[dwDev_num] = 0;
  }
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


/* Exported functions ------------------------------------------------------- */

/**
* @brief :
* @param :
* @retval:
*/
void Pump_Device_Init(void)
{
  uint32_t dwCnt;
  
  for(dwCnt = 0; dwCnt < TB6617_DEVICE_MAXCNT-1; dwCnt++)
  {
    mcrFP_TB6617_STOP(dwCnt);
    mcrFP_TB6617_ENABLE(dwCnt);
  }
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t Pump_Running_Time_Loading(uint32_t dwDev_num, uint32_t dwRungingTime_msec)
{
  if(dwDev_num >= TB6617_DEVICE_MAXCNT) return -1;
  
  g_dwPumpRunningTime[dwDev_num] = dwRungingTime_msec;
  
  return 0;
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t Pump_Single_Run(uint32_t dwDev_num, uint32_t dwDir, uint32_t time)
{
  if(dwDev_num >= TB6617_DEVICE_MAXCNT) return -1;
  
 /// pp_tim_sum(dwDev_num, time);
  
  if(dwDir > nPUMP_RUN_DIR_CW)
  {
    mcrFP_TB6617_CCW(dwDev_num-1);
  }
  else
  {
    mcrFP_TB6617_CW(dwDev_num-1);
  }
  
  Pump_SystickTimerSet(dwDev_num);


  return 0;
}


int32_t Pump_Single_Run_Time(uint32_t dwDev_num, uint32_t dwDir, uint32_t time)
{
  if(dwDev_num >= TB6617_DEVICE_MAXCNT) return -1;
  
  if(dwDir > nPUMP_RUN_DIR_CW)
  {
    mcrFP_TB6617_CCW(dwDev_num);
  }
  else
  {
    mcrFP_TB6617_CW(dwDev_num);
  }
  
  Pump_SystickTimerSet(dwDev_num);


  return 0;
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t Pump_Single_Break(uint32_t dwDev_num)
{
  if(dwDev_num >= TB6617_DEVICE_MAXCNT) return -1;
  
  mcrFP_TB6617_BREAK(dwDev_num-1);
  
  Pump_RunningTimeAdder(dwDev_num-1);
    
  return 0;
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t Pump_Single_Stop(uint32_t dwDev_num)
{
  if(dwDev_num >= TB6617_DEVICE_MAXCNT) return -1;
  
  mcrFP_TB6617_STOP(dwDev_num-1);
  
  Pump_RunningTimeAdder(dwDev_num-1);
    
  return 0;
}


/**
* @brief :
* @param :
* @retval:
*/
int32_t Pump_Single_Duration(uint32_t dwDev_num, uint32_t dwDir, uint32_t dwDuration_msec)
{
  if(Pump_Single_Run(dwDev_num, dwDir,dwDuration_msec) != 0 ) return -1;
  
  mcrFP_PUMP_DELAY_MS(dwDuration_msec);
  
  if(Pump_Single_Break(dwDev_num) != 0) return -1;
  
  return 0;
}
/**
* @brief :
* @param :
* @retval:
*/
int32_t Pump_Multi_Run(uint32_t dwDev_BitCheck, uint32_t dwDir)
{
  uint32_t dwCnt;
  uint32_t dwCheck = 0;
  
  for(dwCnt = 0; dwCnt < TB6617_DEVICE_MAXCNT ; dwCnt++)
  {
    dwCheck = (0x00000001 << dwCnt) & dwDev_BitCheck;
    if(dwCheck)
    {
      Pump_Single_Run(dwCnt, dwDir,0);
    }
  }
  
  return 0;
}
/**
* @brief :
* @param :
* @retval:
*/
int32_t Pump_Multi_Break(uint32_t dwDev_BitCheck)
{
  uint32_t dwCnt;
  uint32_t dwCheck = 0;
  
  for(dwCnt = 0; dwCnt < TB6617_DEVICE_MAXCNT ; dwCnt++)
  {
    dwCheck = (0x00000001 << dwCnt) & dwDev_BitCheck;
    if(dwCheck)
    {
      Pump_Single_Break(dwCnt);
    }
  }
  
  return 0;
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t Pump_Multi_Stop(uint32_t dwDev_BitCheck)
{
  uint32_t dwCnt;
  uint32_t dwCheck = 0;
  
  for(dwCnt = 0; dwCnt < TB6617_DEVICE_MAXCNT ; dwCnt++)
  {
    dwCheck = (0x00000001 << dwCnt) & dwDev_BitCheck;
    if(dwCheck)
    {
      Pump_Single_Stop(dwCnt);
    }
  }
  
  return 0;
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t Pump_Multi_Duration(uint32_t dwDev_BitCheck, uint32_t dwDir, uint32_t dwDuration_msec)
{
  Pump_Multi_Run(dwDev_BitCheck, dwDir);
  mcrFP_PUMP_DELAY_MS(dwDuration_msec);
  Pump_Multi_Break(dwDev_BitCheck);
  
  return 0;
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
