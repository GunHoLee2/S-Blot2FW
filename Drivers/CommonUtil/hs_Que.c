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
#include "hs_Que.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/



/* Private function  ---------------------------------------------------------*/
/**
* @brief :
* @param :
* @retval:
*/

/* Exported functions ------------------------------------------------------- */

// HS Que Code ---------------------------------------------------------------
/**
* @brief :
* @param :
* @retval:
*/
int32_t dwHsQue_Put(hsQueData_Type *p_hsQue, void *p_Data)
{
  uint32_t dwStart_Index = 0;
  uint32_t dwDataCopyCnt = 0;
  uint8_t *p_DataNowTarget = 0;
  
  if( ((p_hsQue->dwRear + 1)%(p_hsQue->dwQueMaxSize) ) == (p_hsQue->dwFront) ) return 1;
  
  dwStart_Index = (p_hsQue->dwRear)  * (p_hsQue->dwDataTypeSize);
  
  p_DataNowTarget = &(p_hsQue->p_uData[dwStart_Index]);
  
  for(dwDataCopyCnt = 0; dwDataCopyCnt < (p_hsQue->dwDataTypeSize); dwDataCopyCnt++)
  {
    p_DataNowTarget[dwDataCopyCnt] = ((uint8_t *)p_Data)[dwDataCopyCnt];
  }
  
  p_hsQue->dwRear = (p_hsQue->dwRear + 1)%(p_hsQue->dwQueMaxSize);
  
  return 0;
}
/**
* @brief :
* @param :
* @retval:
*/
int32_t dwHsQue_Get(hsQueData_Type *p_hsQue, void *p_Data)
{
  uint32_t dwStart_Index = 0;
  uint32_t dwDataCopyCnt = 0;
  uint8_t *p_DataNowTarget = 0;
  
  if( (p_hsQue->dwRear) == (p_hsQue->dwFront) ) return 1;
  
  dwStart_Index = (p_hsQue->dwFront)  * (p_hsQue->dwDataTypeSize);
  
  p_DataNowTarget = &(p_hsQue->p_uData[dwStart_Index]);
  
  for(dwDataCopyCnt = 0; dwDataCopyCnt < (p_hsQue->dwDataTypeSize); dwDataCopyCnt++)
  {
    ((uint8_t *)p_Data)[dwDataCopyCnt] = p_DataNowTarget[dwDataCopyCnt];
  }
  
  p_hsQue->dwFront = (p_hsQue->dwFront + 1)%(p_hsQue->dwQueMaxSize);
  
  return 0;
}



/**
* @brief :
* @param :
* @retval:
*/
int32_t dwHsQue_Config(hsQueData_Type *p_HsQue, void * p_QueBuffer, uint32_t dwDataTypeSize, uint32_t dwQueMaxSize)
{
  p_HsQue->p_uData = (uint8_t *)p_QueBuffer;
  p_HsQue->dwFront = 0;
  p_HsQue->dwRear = 0;
  
  p_HsQue->dwDataTypeSize = dwDataTypeSize;
  p_HsQue->dwQueMaxSize = dwQueMaxSize;
  
  return 0;
}


/**
* @brief :
* @param :
* @retval:
*/
int32_t HsQueGetTimeLimit(hsQueData_Type *p_hsQue, void *p_Data, uint32_t dwTimeLimit_us)
{
  uint32_t dwCnt = 0;
  int32_t dwCheck = 0;
  
  for(;dwCnt < dwTimeLimit_us; dwCnt++)
  {
    dwCheck = dwHsQue_Get(p_hsQue, p_Data);
    if(dwCheck == 0) break;
  }
  
  return dwCheck;
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t HsQuePutTimeLimit(hsQueData_Type *p_hsQue, void *p_Data, uint32_t dwTimeLimit_us)
{
  uint32_t dwCnt = 0;
  int32_t dwCheck = 0;
  
  for(;dwCnt < dwTimeLimit_us; dwCnt++)
  {
    dwCheck = dwHsQue_Put(p_hsQue, p_Data);
    if(dwCheck == 0) break;
  }
  
  return dwCheck;
}


/**
* @brief :
* @param :
* @retval:
*/
void HsQueGetHold(hsQueData_Type *p_hsQue, void *p_Data)
{
  int32_t dwCheck = 0;
  
  while(1)
  {
    dwCheck = dwHsQue_Get(p_hsQue, p_Data);
    if(dwCheck == 0) break;
  }
}

/**
* @brief :
* @param :
* @retval:
*/
void HsQuePutHold(hsQueData_Type *p_hsQue, void *p_Data)
{
  int32_t dwCheck = 0;
  
  while(1)
  {
    dwCheck = dwHsQue_Put(p_hsQue, p_Data);
    if(dwCheck == 0) break;
  }
}


//---------------------------------------------------------------------------




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
