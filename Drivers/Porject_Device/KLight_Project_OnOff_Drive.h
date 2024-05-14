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
#ifndef __KLIGHT_PRJ_ONOFF_DRV_H
#define __KLIGHT_PRJ_ONOFF_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


   
   
void KLight_BathPower_OnOff(uint32_t dwOn_nOff);
int32_t KLight_Bath_Detect_Pin1(void);
int32_t KLight_Bath_Detect_Pin2(void);

void KLight_Buzzer_OnOff(uint32_t dwDuration_ms, uint32_t dwRePlayCnt);

void KLight_Fan_OnOff(uint32_t dwOn_nOff);

void KLight_Heater_OnOff(uint32_t dwOn_nOff);



   
#ifdef __cplusplus
}
#endif


#endif /* __KLIGHT_PRJ_ONOFF_DRV_H*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
