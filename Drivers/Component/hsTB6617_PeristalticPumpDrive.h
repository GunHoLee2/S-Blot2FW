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
#ifndef __HS_TB6617_DRIVE_H
#define __HS_TB6617_DRIVE_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#define nPUMP_RUN_DIR_CW        0
#define nPUMP_RUN_DIR_CCW       1
   
void Pump_Device_Init(void);
int32_t Pump_Running_Time_Loading(uint32_t dwDev_num, uint32_t dwRungingTime_msec);

int32_t Pump_Single_Run(uint32_t dwDev_num, uint32_t dwDir, uint32_t time);
int32_t Pump_Single_Run_Time(uint32_t dwDev_num, uint32_t dwDir, uint32_t time);
int32_t Pump_Single_Break(uint32_t dwDev_num);
int32_t Pump_Single_Stop(uint32_t dwDev_num);

int32_t Pump_Single_Duration(uint32_t dwDev_num, uint32_t dwDir, uint32_t dwDuration_msec);

int32_t Pump_Multi_Run(uint32_t dwDev_BitCheck, uint32_t dwDir);
int32_t Pump_Multi_Break(uint32_t dwDev_BitCheck);
int32_t Pump_Multi_Stop(uint32_t dwDev_BitCheck);

int32_t Pump_Multi_Duration(uint32_t dwDev_BitCheck, uint32_t dwDir, uint32_t dwDuration_msec);



   
#ifdef __cplusplus
}
#endif


#endif /* __HS_TB6617_DRIVE_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
