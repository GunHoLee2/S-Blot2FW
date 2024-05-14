/**
  ******************************************************************************
  * @file    
  * @author  
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  *
  ******************************************************************************
  */  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HS_DRV8825_DRIVE_H
#define __HS_DRV8825_DRIVE_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


#define nDRV88xx_RESET  0
#define nDRV88xx_CW     2
#define nDRV88xx_CCW    1

#define nDRV88xx_STA_NON       -1
#define nDRV88xx_STA_IDLE       0
#define nDRV88xx_STA_HOMMING    1 
#define nDRV88xx_STA_CWMV       2
#define nDRV88xx_STA_CCWMV      3
#define nDRV88xx_STA_ERR        4   

#define nDRV88xx_STOP           1
#define nDRV88xx_PAUSE          2
   
   
   
typedef struct
{
  int32_t dwPos;
  int32_t dwTargetPos;
  int32_t dwState;              //-1 not Homming, 0 IDLE, 1 Hommming, 2 cw move, 3 ccw move
  int32_t dwFault;
  int32_t dwStateCWCCW;         //0 리셋 상태 , 1 CW 방향, 2 CCW 방향 이었음.
  int32_t dwMotion;             //0 Noraml, 1 CW --> CCW, CCW --> CW
  int32_t dwStopPuase;               // 0 , 1 stop
  
  uint32_t dwMiniumSpeed;
  uint32_t dwMaxSpeed;
  uint32_t dwAccSpeed;
  uint32_t dwDecSpeed;
  uint32_t dwMaxPulseCnt;
  uint32_t dwAccPulseCnt;
  uint32_t dwDecPulseCnt;
  
}DRV88xx_State_Type;
   

  
void DRV88xx_Init(void);
int32_t DRV88xx_Homming(uint32_t dwDev_num, uint32_t dwSpeed_pps);
int32_t DRV88xx_AbsMove(uint32_t dwDev_num, uint32_t dwSpeed_pps, int32_t dwABS_Pos);

int32_t DRV88xx_Position(uint32_t dwDev_num);
void DRV88xx_StopPause(uint32_t dwDev_num, uint32_t dwMod);
int32_t DRV88xx_State(uint32_t dwDev_num);

int32_t DRV88xx_HomeCheck(uint32_t dwDev_num);






#ifdef __cplusplus
}
#endif


#endif /* __HS_DRV8825_DRIVE_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
