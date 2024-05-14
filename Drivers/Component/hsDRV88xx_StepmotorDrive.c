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
#include "hsDRV88xx_StepmotorDrive.h"
#include "hs_Common_Utill.h"

//#include "Pump_Ctrl_v2xx_Config.h"
#include "hsBSP_MCU_Config.h" 
#include "stm32f1xx_hal_tim.h"


#define DRV88xx_DEVICE_MAXCNT           3
#define DRV88xx_HOME_DETECTION         0
#define DRV88xx_DECAY_PULSE             100
#define DRV88xx_HOME_PULSE_LIMIE     1000000
#define DRV88xx_MOVE_PULSE_LIMIE     1000000

/* Private variables ---------------------------------------------------------*/
DRV88xx_State_Type g_tDRV88xx_Axis[DRV88xx_DEVICE_MAXCNT];




//-----------------------------------------------------------------------------
//BSP Aline
const uint32_t g_dwFpDRV88xx_StandBy    = (uint32_t)BSP_DRV8825_StandBy;        //void BSP_DRV8825_StandBy(uint32_t dwDev_num);
const uint32_t g_dwFpDRV88xx_DIR        = (uint32_t)BSP_DRV8825_DIR;            //void BSP_DRV8825_DIR(uint32_t dwDev_num, uint32_t dwCCW_nCW);
const uint32_t g_dwFpDRV88xx_DECAY      = (uint32_t)BSP_DRV8825_DECAY;          //void BSP_DRV8825_DECAY(uint32_t dwDev_num, uint32_t dwFast_nLow);
const uint32_t g_dwFpDRV88xx_RESET      = (uint32_t)BSP_DRV8825_RESET;          //void BSP_DRV8825_RESET(uint32_t dwDev_num);
const uint32_t g_dwFpDRV88xx_STEP       = (uint32_t)BSP_DRV8825_STEP;           //void BSP_DRV8825_STEP(uint32_t dwDev_num, uint32_t dwPulse_us);
const uint32_t g_dwFpDRV88xx_FAULT      = (uint32_t)BSP_DRV8825_Fault_Check;    //int32_t BSP_DRV8825_Fault_Check(uint32_t dwDev_num);
const uint32_t g_dwFpDRV88xx_HOME       = (uint32_t)BSP_DRV8825_Home_Check;     //int32_t BSP_DRV8825_Home_Check(uint32_t dwDev_num);
 const uint32_t g_dwFpDRV88xx_ENABLE =    (uint32_t)BSP_DRV8825_ENABLE;

const uint32_t g_dwFpDRV88xx_Delay_us   = (uint32_t)dwHsSW_Delay_us;

/* macro ----------------------------------------------------------------------*/
#define mcrFP_DRV88XX_STANDBY           ((void(*)(uint32_t))g_dwFpDRV88xx_StandBy)
#define mcrFP_DRV88XX_DIR               ((void(*)(uint32_t, uint32_t))g_dwFpDRV88xx_DIR)
#define mcrFP_DRV88XX_DECAY             ((void(*)(uint32_t, uint32_t))g_dwFpDRV88xx_DECAY)
#define mcrFP_DRV88XX_RESET             ((void(*)(uint32_t))g_dwFpDRV88xx_RESET)
#define mcrFP_DRV88XX_STEP              ((void(*)(uint32_t, uint32_t))g_dwFpDRV88xx_STEP)
#define mcrFP_DRV88XX_FAULT             ((int32_t(*)(uint32_t))g_dwFpDRV88xx_FAULT)
#define mcrFP_DRV88XX_HOME              ((int32_t(*)(uint32_t))g_dwFpDRV88xx_HOME)
#define mcrFP_DRV88XX_ENABLE              ((void(*)(uint32_t, uint32_t))g_dwFpDRV88xx_ENABLE)

#define mcrFP_DRV88xx_DELAY_US          ((void(*)(uint32_t))g_dwFpDRV88xx_Delay_us)


/* Private function prototypes -----------------------------------------------*/



/* Private function  ---------------------------------------------------------*/

/**
* @brief :
* @param :
* @retval:
*/
int32_t DRV88xx_HomeCheck(uint32_t dwDev_num)
{
    return mcrFP_DRV88XX_HOME(dwDev_num);
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
void DRV88xx_Error_Set(uint32_t dwDev_num)
{
  g_tDRV88xx_Axis[dwDev_num].dwState = nDRV88xx_STA_ERR;
}


/* Exported functions ------------------------------------------------------- */

/**
* @brief :
* @param :
* @retval:
#define nDRV88xx_RESET  0
#define nDRV88xx_CW     1
#define nDRV88xx_CCW    2

#define nDRV88xx_STA_NON       -1
#define nDRV88xx_STA_IDLE       0
#define nDRV88xx_STA_HOMMING    1 
#define nDRV88xx_STA_CWMV       2
#define nDRV88xx_STA_CCWMV      3
#define nDRV88xx_STA_ERR        4   

*/
void DRV88xx_Init(void)
{
  uint32_t dwCnt;
  
  for(dwCnt = 0; dwCnt < DRV88xx_DEVICE_MAXCNT;dwCnt++)
  {
    mcrFP_DRV88XX_STANDBY(dwCnt);
    mcrFP_DRV88XX_ENABLE(dwCnt,1);
    
    //g_tDRV88xx_Axis[dwCnt].dwPos = 0;
    //g_tDRV88xx_Axis[dwCnt].dwTargetPos = 0;
    //g_tDRV88xx_Axis[dwCnt].dwFault = 0;
    //g_tDRV88xx_Axis[dwCnt].dwState = nDRV88xx_STA_NON;
  }
}

/**
* @brief :
* @param :
* @retval: -1 parameter Fail, 
*/
int32_t DRV88xx_Homming(uint32_t dwDev_num, uint32_t dwSpeed_pps)
{
  uint32_t dwPulse_Delay = 0;
  int32_t dwHome_Check = 0;
  int32_t dwFault_Check = 0;
  int32_t dwDev_State = 0;
  uint32_t dwDevCCW_nCW = 0;
  uint32_t dwPulseCnt = 0;
  uint32_t dwDecay_block = 0;
  
  if( (dwSpeed_pps < 1) || (dwDev_num >= DRV88xx_DEVICE_MAXCNT) ) return -1; // parameter fail
  
  //dwPulse_Delay = 1000000/(dwSpeed_pps*3);
  
    if(dwSpeed_pps>375)
    dwSpeed_pps=375;
   dwPulse_Delay=(60*1000000)/(dwSpeed_pps*6400);
   dwPulse_Delay=dwPulse_Delay/5;
   if(dwPulse_Delay<5)
     dwPulse_Delay=5;
  
  mcrFP_DRV88XX_RESET(dwDev_num);
  
  dwHome_Check = DRV88xx_HomeCheck(dwDev_num);
  
  
  if(dwHome_Check != DRV88xx_HOME_DETECTION)
  {
    //Homming 구동은 CCW로 구동하면 Home 위치를 찾는다.
    if(g_tDRV88xx_Axis[dwDev_num].dwMotion == 1)
    {
      dwDevCCW_nCW = 1; 
    }
    else
    {
      dwDevCCW_nCW = 0;
    }
    
    mcrFP_DRV88XX_DIR(dwDev_num, dwDevCCW_nCW);
    mcrFP_DRV88XX_DECAY(dwDev_num, 1);
    mcrFP_DRV88XX_ENABLE(dwDev_num,0);
    while(1)
    {
      mcrFP_DRV88XX_STEP(dwDev_num, dwPulse_Delay);
      
      dwPulseCnt++;
      
      if(dwPulseCnt >= DRV88xx_DECAY_PULSE)
      {
        if(dwDecay_block == 0)
        {
          mcrFP_DRV88XX_DECAY(dwDev_num, 1);
          dwDecay_block = 1; 
        }
      }
      if(dwPulseCnt >= DRV88xx_HOME_PULSE_LIMIE)
      {
        dwDev_State = 1;
        goto DRV88XX_HOME_MOV_STAT_CHECK;
      }
      
      dwFault_Check = mcrFP_DRV88XX_FAULT(dwDev_num);
      if(dwFault_Check)
      {
        dwDev_State = 2;
        goto DRV88XX_HOME_MOV_STAT_CHECK;
      }
      
      dwHome_Check = mcrFP_DRV88XX_HOME(dwDev_num);
      if(dwHome_Check ==  DRV88xx_HOME_DETECTION)
      {
        dwHome_Check = DRV88xx_HomeCheck(dwDev_num);
        if(dwHome_Check == DRV88xx_HOME_DETECTION) break;;
      }
      while(g_tDRV88xx_Axis[dwDev_num].dwStopPuase == 2)
      {
        asm("NOP");
      }
      
      if(g_tDRV88xx_Axis[dwDev_num].dwStopPuase == 1) goto DRV88XX_HOME_MOV_STAT_CHECK;
      
    }
  }

  //Home위치를 확인 했으면 CW로 이동 Home 위치에서 1 Puse 앞에 위치 한다.
  if(g_tDRV88xx_Axis[dwDev_num].dwMotion == 1)
  {
    dwDevCCW_nCW = 0; 
  }
  else
  {
    dwDevCCW_nCW = 1; 
  }
  
  mcrFP_DRV88XX_RESET(dwDev_num);
  mcrFP_DRV88XX_DIR(dwDev_num, dwDevCCW_nCW);
  mcrFP_DRV88XX_DECAY(dwDev_num, 1);
  mcrFP_DRV88XX_ENABLE(dwDev_num,0);
  
  dwPulseCnt = 0;
  dwDecay_block = 0;
  
  while(1)
  {
    mcrFP_DRV88XX_STEP(dwDev_num, dwPulse_Delay);
    
    dwPulseCnt++;
    
    if(dwPulseCnt >= DRV88xx_DECAY_PULSE)
    {
      if(dwDecay_block == 0)
      {
        mcrFP_DRV88XX_DECAY(dwDev_num, 1);
        dwDecay_block = 1; 
      }
    }
    
    if(dwPulseCnt >= DRV88xx_HOME_PULSE_LIMIE)
    {
      //dwDev_State = 1;
      goto DRV88XX_HOME_MOV_STAT_CHECK;
    }
    
    dwFault_Check = mcrFP_DRV88XX_FAULT(dwDev_num);
    if(dwFault_Check)
    {
      dwDev_State = 2;
      goto DRV88XX_HOME_MOV_STAT_CHECK;
    }
    
   dwHome_Check = mcrFP_DRV88XX_HOME(dwDev_num);
    if(dwHome_Check !=  DRV88xx_HOME_DETECTION)
    {
      dwHome_Check = DRV88xx_HomeCheck(dwDev_num);
      if(dwHome_Check != DRV88xx_HOME_DETECTION) break;;
    }

    while(g_tDRV88xx_Axis[dwDev_num].dwStopPuase == 2)
    {
      asm("NOP");
    }
    
    if(g_tDRV88xx_Axis[dwDev_num].dwStopPuase == 1) break;
    
    
  }    

DRV88XX_HOME_MOV_STAT_CHECK:
  
  switch(dwDev_State)
  {
  case 0:
    
      g_tDRV88xx_Axis[dwDev_num].dwPos = 0;
      g_tDRV88xx_Axis[dwDev_num].dwTargetPos = 0;
      g_tDRV88xx_Axis[dwDev_num].dwFault = 0;
      g_tDRV88xx_Axis[dwDev_num].dwState = 0;
      g_tDRV88xx_Axis[dwDev_num].dwStateCWCCW = nDRV88xx_CW;
      mcrFP_DRV88XX_RESET(dwDev_num);
      if(dwDev_num!=1)
      mcrFP_DRV88XX_ENABLE(dwDev_num,1);
      
      
      break;
  case 1: //Time out
    DRV88xx_Error_Set(dwDev_num);
    return 1;
    //break;
  case 2: //Device Fault Error
    g_tDRV88xx_Axis[dwDev_num].dwFault = 1;
    DRV88xx_Error_Set(dwDev_num);
    return 2;
    //break;
  default:
    return 3;
    //break;
  }
  
  return 0;
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t DRV88xx_AbsMove(uint32_t dwDev_num, uint32_t dwSpeed_pps, int32_t dwABS_Pos)
{
  uint32_t dwPulse_Delay = 0;
  int32_t dwFault_Check = 0;
  int32_t dwDev_State = 0;
  uint32_t dwDevCCW_nCW = 0;
  uint32_t dwDevDIR_Check = 0;  
  uint32_t dwPulseCnt = 0;
  uint32_t dwDecay_block = 0;
  //Time2_It_Stop();
  //-------------------------------------------------------
  if( (dwSpeed_pps < 1) || (dwDev_num >= DRV88xx_DEVICE_MAXCNT) ) return -1; // parameter fail
  if( (g_tDRV88xx_Axis[dwDev_num].dwState != nDRV88xx_STA_IDLE) || (g_tDRV88xx_Axis[dwDev_num].dwFault == 1)) 
  {
    dwDev_State = 1;
    goto DRV88XX_ABS_MOV_STAT_CHECK;
  }
  g_tDRV88xx_Axis[dwDev_num].dwPos=0;
  //-------------------------------------------------------
  
  //dwPulse_Delay = 1000000/(dwSpeed_pps*2);
  if(dwSpeed_pps>375)
    dwSpeed_pps=375;
   dwPulse_Delay=(60*1000000)/(dwSpeed_pps*6400);
   dwPulse_Delay=dwPulse_Delay/5;
   if(dwPulse_Delay<5)
     dwPulse_Delay=5;
   
 
  if(g_tDRV88xx_Axis[dwDev_num].dwPos <dwABS_Pos)
  {
    dwDevDIR_Check = nDRV88xx_CW;
  }
  else if(g_tDRV88xx_Axis[dwDev_num].dwPos > dwABS_Pos)
  {
    dwDevDIR_Check = nDRV88xx_CCW;
  }
  else
  {
    dwDev_State = 0;
    goto DRV88XX_ABS_MOV_STAT_CHECK;
  }
  
  g_tDRV88xx_Axis[dwDev_num].dwTargetPos = dwABS_Pos;
  //-------------------------------------------------------
  
  if(g_tDRV88xx_Axis[dwDev_num].dwMotion == 1)
  {
    if(dwDevDIR_Check == nDRV88xx_CW) dwDevCCW_nCW = 1;
    else dwDevCCW_nCW = 0;
  }
  else
  {
    if(dwDevDIR_Check == nDRV88xx_CW) dwDevCCW_nCW = 0;
    else dwDevCCW_nCW = 1;
  }
  
  if(dwDevDIR_Check != g_tDRV88xx_Axis[dwDev_num].dwStateCWCCW)
  {
    mcrFP_DRV88XX_RESET(dwDev_num);
    mcrFP_DRV88xx_DELAY_US(10000);
    g_tDRV88xx_Axis[dwDev_num].dwStateCWCCW = dwDevDIR_Check;
  }
  //-------------------------------------------------------

  mcrFP_DRV88XX_DIR(dwDev_num, dwDevCCW_nCW);
  mcrFP_DRV88XX_DECAY(dwDev_num, 1);
  mcrFP_DRV88XX_ENABLE(dwDev_num,0);
  
  dwPulseCnt = 0;
  dwDecay_block = 0;
  
  while(1)
  {
    mcrFP_DRV88XX_STEP(dwDev_num, dwPulse_Delay);
    
    dwPulseCnt++;
   
    if(dwPulseCnt >= DRV88xx_DECAY_PULSE)
    {
      if(dwDecay_block == 0)
      {
        mcrFP_DRV88XX_DECAY(dwDev_num, 1);
        dwDecay_block = 1; 
      }
    }
    
    if(dwPulseCnt >= DRV88xx_MOVE_PULSE_LIMIE)
    {
      dwDev_State = 2;
      goto DRV88XX_ABS_MOV_STAT_CHECK;
    }
    
    dwFault_Check = mcrFP_DRV88XX_FAULT(dwDev_num);
    if(dwFault_Check)
    {
      dwDev_State = 3;
      goto DRV88XX_ABS_MOV_STAT_CHECK;
    }
    
    if(dwDevDIR_Check == nDRV88xx_CW)
    {
      g_tDRV88xx_Axis[dwDev_num].dwPos = g_tDRV88xx_Axis[dwDev_num].dwPos + 1;
      if(g_tDRV88xx_Axis[dwDev_num].dwPos >= g_tDRV88xx_Axis[dwDev_num].dwTargetPos) break;
    }
    else
    {
      g_tDRV88xx_Axis[dwDev_num].dwPos = g_tDRV88xx_Axis[dwDev_num].dwPos - 1;
      if(g_tDRV88xx_Axis[dwDev_num].dwPos <= g_tDRV88xx_Axis[dwDev_num].dwTargetPos) break;
    }

    while(g_tDRV88xx_Axis[dwDev_num].dwStopPuase == 2)
    {
      asm("NOP");
    }
    
    if(g_tDRV88xx_Axis[dwDev_num].dwStopPuase == 1) break;

   
  }
  

DRV88XX_ABS_MOV_STAT_CHECK:
  
  switch(dwDev_State)
  {
  case 0:
    mcrFP_DRV88XX_DECAY(dwDev_num, 1);
    mcrFP_DRV88XX_RESET(dwDev_num);
    //Time2_It_Start();
    if(dwDev_num!=1)
    mcrFP_DRV88XX_ENABLE(dwDev_num,1);
    break;
  case 1:
    
    return dwDev_State;
    //break;
  case 2:
    DRV88xx_Error_Set(dwDev_num);
    return dwDev_State;
  case 3:
    g_tDRV88xx_Axis[dwDev_num].dwFault = 1;
    DRV88xx_Error_Set(dwDev_num);
    
    return dwDev_State;
  default:
    return dwDev_State;
  }
  
  return 0;
}
/**
* @brief :
* @param :
* @retval:
*/
int32_t DRV88xx_AbsMove_Forced(uint32_t dwDev_num, uint32_t dwSpeed_pps, int32_t dwABS_Pos)
{
  
  return 0;
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t DRV88xx_Position(uint32_t dwDev_num)
{
  return g_tDRV88xx_Axis[dwDev_num].dwPos;
}
/**
* @brief :
* @param :
* @retval:
*/
void DRV88xx_StopPause(uint32_t dwDev_num, uint32_t dwMod)
{
  g_tDRV88xx_Axis[dwDev_num].dwStopPuase = dwMod;
}
/**
* @brief :
* @param :
* @retval:
*/
int32_t DRV88xx_State(uint32_t dwDev_num)
{
  return g_tDRV88xx_Axis[dwDev_num].dwState;
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
