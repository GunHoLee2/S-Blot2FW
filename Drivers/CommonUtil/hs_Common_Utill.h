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
#ifndef __HS_COMMON_UTILL_H
#define __HS_COMMON_UTILL_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

   

   
int32_t dwHsSW_Delay_us(uint32_t dwDelay_us);
int32_t dwHsSW_Delay_ms(uint32_t dwDelay_ms);

void vSwap_u32bit(uint32_t *p_A, uint32_t *p_B);
void vSwap_u8bit(uint8_t *p_A, uint8_t *p_B);
void vHsSwapUpSort_u32bit(uint32_t *p_Data, uint32_t Len);

#ifdef __cplusplus
}
#endif


#endif /* __HS_COMMON_UTILL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
