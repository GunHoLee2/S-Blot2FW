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


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/



/* Private function  ---------------------------------------------------------*/


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
int32_t dwHsSW_Delay_us(uint32_t dwDelay_us)
{
  uint32_t dwCnt_0;
  
  for(dwCnt_0 = 0; dwCnt_0 < dwDelay_us; dwCnt_0++)
  {
//    for(dwCnt_1 = 0; dwCnt_1 < 2; dwCnt_1++)
//    {
       //sub_root();
      asm("NOP");
     
//   }
    
  }
 
/*  
for(dwCnt_0 = 0; dwCnt_0 < dwDelay_us; dwCnt_0++)
  {
    for(dwCnt_1 = 0; dwCnt_1 < 12; dwCnt_1++)
    {
      asm("NOP");
    }
      main_root();
  }*/
  return 0;
}

/**
* @brief :
* @param :
* @retval:
*/
int32_t dwHsSW_Delay_ms(uint32_t dwDelay_ms)
{
  uint32_t dwCnt_0, dwCnt_1;
  
  for(dwCnt_0 = 0; dwCnt_0 < dwDelay_ms; dwCnt_0++)
  {
    for(dwCnt_1 = 0; dwCnt_1 < 5000; dwCnt_1++)
    {
      asm("NOP");
    }
  }
  
  return 0;
}


/**
* @brief :
* @param :
* @retval:
*/
void vHsSwapUpSort_u32bit(uint32_t *p_Data, uint32_t Len)
{
  int dwCnt1, dwCnt2;
  
  for(dwCnt1 = 0; dwCnt1 < Len ; dwCnt1++)
  {
    for(dwCnt2 = 0; dwCnt2 < Len ; dwCnt2++)
    {
      if(p_Data[dwCnt1] < p_Data[dwCnt2])
      {
        vSwap_u32bit(&p_Data[dwCnt1], &p_Data[dwCnt2]);
      }
      else 
        continue;
    }
  }
  
}
/**
* @brief :
* @param :
* @retval:
*/
void vSwap_u32bit(uint32_t *p_A, uint32_t *p_B)
{
  uint32_t dwSwap;
  
  dwSwap = *p_A;
  *p_A = *p_B;
  *p_B = dwSwap; 
}

/**
* @brief :
* @param :
* @retval:
*/
void vSwap_u8bit(uint8_t *p_A, uint8_t *p_B)
{
   uint8_t dwSwap;
  
  dwSwap = *p_A;
  *p_A = *p_B;
  *p_B = dwSwap; 
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




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
