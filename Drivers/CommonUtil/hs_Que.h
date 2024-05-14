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
#ifndef __HS_QUE_H
#define __HS_QUE_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/

// Device include -----------------------------------------------------------
#include "stm32f1xx_hal.h"

//---------------------------------------------------------------------------
      
typedef struct
{
  uint8_t *p_uData;
  int32_t dwFront;
  int32_t dwRear;
  uint32_t dwDataTypeSize;
  uint32_t dwQueMaxSize;
}hsQueData_Type;





int32_t dwHsQue_Config(hsQueData_Type *p_HsQue, void * p_QueBuffer, uint32_t dwDataTypeSize, uint32_t dwQueMaxSize);
  
int32_t dwHsQue_Put(hsQueData_Type *p_hsQue, void *p_Data);
int32_t dwHsQue_Get(hsQueData_Type *p_hsQue, void *p_Data);
int32_t HsQueGetTimeLimit(hsQueData_Type *p_hsQue, void *p_Data, uint32_t dwTimeLimit_us);
int32_t HsQuePutTimeLimit(hsQueData_Type *p_hsQue, void *p_Data, uint32_t dwTimeLimit_us);
void HsQueGetHold(hsQueData_Type *p_hsQue, void *p_Data);
void HsQuePutHold(hsQueData_Type *p_hsQue, void *p_Data);


#ifdef __cplusplus
}
#endif


#endif /* __HS_QUE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
