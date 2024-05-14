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
#include "usb_uart.h"
//#include "hsBSP_MCU_Config.h"
#include "hs_Common_Utill.h"


#include "stm32f1xx_it.h"


#define __HS_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __HS_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __HS_BSP_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __HS_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */ 
#define __HS_BSP_VERSION        ((__HS_BSP_VERSION_MAIN << 24)\
                                            |(__HS_BSP_VERSION_SUB1 << 16)\
                                            |(__HS_BSP_VERSION_SUB2 << 8 )\
                                            |(__HS_BSP_VERSION_RC))




//extern void Error_Handler(void);
/* Private variables ---------------------------------------------------------*/
//
__IO uint32_t g_dwHW_SystickTimer;
uint32_t g_dwHW_TimeSet[10];
uint32_t g_dwSystemHW_TimerBreak;

//Callback 
uint32_t g_dwFpCallBack_Bank[10];

//Device 
TIM_HandleTypeDef hWORK_TIM_Handle;
TIM_HandleTypeDef sEV_TIM_Handle;

UART_HandleTypeDef hUSB_COM_Handle;
UART_HandleTypeDef hFLASHER_COM_Handle;
UART_HandleTypeDef hLCD_COM_Handle;
UART_HandleTypeDef hRS485_COM_Handle;


ADC_HandleTypeDef hTHERM_ADC_Handle;
I2C_HandleTypeDef hEEPROM_I2C_Handle;
TIM_HandleTypeDef hSERVO_TIM_Handle;

SPI_HandleTypeDef smt;


//IO Soft Mapping

//Pump
const GPIO_TypeDef *gp_PORT_TB6617FNG_PWM[]     = {PUMP1_PWM_GPIO_Port,         PUMP2_PWM_GPIO_Port,            PUMP3_PWM_GPIO_Port,            PUMP4_PWM_GPIO_Port,            PUMP5_PWM_GPIO_Port,            PUMP6_PWM_GPIO_Port,            PUMP7_PWM_GPIO_Port,            PUMP8_PWM_GPIO_Port,     PUMP9_PWM_GPIO_Port};
const uint16_t g_GPIO_Pin_TB6617FNG_PWM[]       = {PUMP1_PWM_Pin,               PUMP2_PWM_Pin,                  PUMP3_PWM_Pin,                  PUMP4_PWM_Pin,                  PUMP5_PWM_Pin,                  PUMP6_PWM_Pin,                  PUMP7_PWM_Pin,                  PUMP8_PWM_Pin,           PUMP9_PWM_Pin};

const GPIO_TypeDef *gp_PORT_TB6617FNG_CW[]      = {PUMP1_IN1_CW_GPIO_Port,      PUMP2_IN1_CW_GPIO_Port,         PUMP3_IN1_CW_GPIO_Port,         PUMP4_IN1_CW_GPIO_Port,         PUMP5_IN1_CW_GPIO_Port,         PUMP6_IN1_CW_GPIO_Port,         PUMP7_IN1_CW_GPIO_Port,         PUMP8_IN1_CW_GPIO_Port,  PUMP9_IN1_CW_GPIO_Port  };
const uint16_t g_GPIO_Pin_TB6617FNG_CW[]        = {PUMP1_IN1_CW_Pin,            PUMP2_IN1_CW_Pin,               PUMP3_IN1_CW_Pin,               PUMP4_IN1_CW_Pin,               PUMP5_IN1_CW_Pin,               PUMP6_IN1_CW_Pin,               PUMP7_IN1_CW_Pin,               PUMP8_IN1_CW_Pin,        PUMP9_IN1_CW_Pin};

const GPIO_TypeDef *gp_PORT_TB6617FNG_CCW[]     = {PUMP1_IN2_CCW_GPIO_Port,     PUMP2_IN2_CCW_GPIO_Port,        PUMP3_IN2_CCW_GPIO_Port,        PUMP4_IN2_CCW_GPIO_Port,        PUMP5_IN2_CCW_GPIO_Port,        PUMP6_IN2_CCW_GPIO_Port,        PUMP7_IN2_CCW_GPIO_Port,        PUMP8_IN2_CCW_GPIO_Port, PUMP9_IN2_CCW_GPIO_Port};
const uint16_t g_GPIO_Pin_TB6617FNG_CCW[]       = {PUMP1_IN2_CCW_Pin,           PUMP2_IN2_CCW_Pin,              PUMP3_IN2_CCW_Pin,              PUMP4_IN2_CCW_Pin,              PUMP5_IN2_CCW_Pin,              PUMP6_IN2_CCW_Pin,              PUMP7_IN2_CCW_Pin,              PUMP8_IN2_CCW_Pin,       PUMP9_IN2_CCW_Pin};

////Drv8825
//const GPIO_TypeDef *gp_PORT_DRV8825_DIR[]       = {DRV8825_DIR_X_AXIS_GPIO_Port,DRV8825_DIR_ASP_GPIO_Port,      DRV8825_DIR_TRAY_GPIO_Port      };
//const uint16_t g_GPIO_Pin_DRV8825_DIR[]         = {DRV8825_DIR_X_AXIS_Pin,      DRV8825_DIR_ASP_Pin,            DRV8825_DIR_TRAY_Pin            };
//
//const GPIO_TypeDef *gp_PORT_DRV8825_STEP[]      = {DRV8825_STEP_X_AXIS_GPIO_Port,DRV8825_STEP_ASP_GPIO_Port,    DRV8825_STEP_TRAY_GPIO_Port     };
//const uint16_t g_GPIO_Pin_DRV8825_STEP[]        = {DRV8825_STEP_X_AXIS_Pin,     DRV8825_STEP_ASP_Pin,           DRV8825_STEP_TRAY_Pin           };
//
//const GPIO_TypeDef *gp_PORT_DRV8825_ENABLE[]    = {DRV8825_ENABLE_X_AXIS_GPIO_Port,DRV8825_ENABLE_ASP_GPIO_Port,DRV8825_ENABLE_TRAY_GPIO_Port   };
//const uint16_t g_GPIO_Pin_DRV8825_ENABLE[]      = {DRV8825_ENABLE_X_AXIS_Pin,   DRV8825_ENABLE_ASP_Pin,         DRV8825_ENABLE_TRAY_Pin         };
//
//const GPIO_TypeDef *gp_PORT_DRV8825_SLEEP[]     = {DRV8825_SLEEP_X_AXIS_GPIO_Port,DRV8825_SLEEP_ASP_GPIO_Port,  DRV8825_SLEEP_TRAY_GPIO_Port    };
//const uint16_t g_GPIO_Pin_DRV8825_SLEEP[]       = {DRV8825_SLEEP_X_AXIS_Pin,    DRV8825_SLEEP_ASP_Pin,          DRV8825_SLEEP_TRAY_Pin          };
//
//const GPIO_TypeDef *gp_PORT_DRV8825_RESET[]     = {DRV8825_RESET_X_AXIS_GPIO_Port,DRV8825_RESET_ASP_GPIO_Port,  DRV8825_RESET_TRAY_GPIO_Port    };
//const uint16_t g_GPIO_Pin_DRV8825_RESET[]       = {DRV8825_RESET_X_AXIS_Pin,    DRV8825_RESET_ASP_Pin,          DRV8825_RESET_TRAY_Pin          };
//
//const GPIO_TypeDef *gp_PORT_DRV8825_DECAY[]     = {DRV8825_DECAY_X_AXIS_GPIO_Port,DRV8825_DECAY_ASP_GPIO_Port,  DRV8825_DECAY_TRAY_GPIO_Port    };
//const uint16_t g_GPIO_Pin_DRV8825_DECAY[]       = {DRV8825_DECAY_X_AXIS_Pin,    DRV8825_DECAY_ASP_Pin,          DRV8825_DECAY_TRAY_Pin          };
//
//const GPIO_TypeDef *gp_PORT_DRV8825_FAULT[]     = {DRV8825_FAULT_X_AXIS_GPIO_Port,ASP_FAULT_GPIO_Port,          DRV8825_FAULT_TRAY_GPIO_Port    };
//const uint16_t g_GPIO_Pin_DRV8825_FAULT[]       = {DRV8825_FAULT_X_AXIS_Pin,    ASP_FAULT_Pin,                  DRV8825_FAULT_TRAY_Pin          };
//
const GPIO_TypeDef *gp_PORT_DRV8825_HOME_SEN[]  = {M2_HOME_GPIO_Port,   M0_HOME_GPIO_Port     };
const uint16_t g_GPIO_Pin_DRV8825_HOME_SEN[]    = {M2_HOME_Pin,         M0_HOME_Pin           };

//Indicator LED
const GPIO_TypeDef *gp_PORT_IndcatorLED[]       = {LED_MCU_CHECK_GPIO_Port,     LED_POWER_GPIO_Port,            LED_ASPDSP_GPIO_Port,           LED_INCUBATION_GPIO_Port,               LED_DRY_GPIO_Port       };
const uint16_t g_GPIO_Pin_IndcatorLED[]         = {LED_MCU_CHECK_Pin,           LED_POWER_Pin,                  LED_ASPDSP_Pin,                 LED_INCUBATION_Pin,                     LED_DRY_Pin             };

//ETC OnOff
const GPIO_TypeDef *gp_PORT_ETC_ONOFF[]         = {BUZZER_GPIO_Port,            FAN_ONOFF_GPIO_Port,            RESERVE_ONOFF_GPIO_Port,        HEATER_ONOFF_GPIO_Port,                 BATH_PIN_ONOFF_GPIO_Port};
const uint16_t g_GPIO_Pin_ETC_ONOFF[]           = {BUZZER_Pin,                  FAN_ONOFF_Pin,                  RESERVE_ONOFF_Pin,              HEATER_ONOFF_Pin,                       BATH_PIN_ONOFF_Pin      };

//ETC Input Check
const GPIO_TypeDef *gp_PORT_ETC_INPUT[]         = {RESERVE_IN_GPIO_Port,        BATH_PIN_1_GPIO_Port,           BATH_PIN_2_GPIO_Port    };
const uint16_t g_GPIO_Pin_ETC_INPUT[]           = {RESERVE_IN_Pin,              BATH_PIN_1_Pin,                 BATH_PIN_2_Pin          };






const UART_HandleTypeDef *gp_UART_Handle[]      = {&hFLASHER_COM_Handle,        &hLCD_COM_Handle,               &hRS485_COM_Handle,             &hUSB_COM_Handle        };

/* Private function prototypes -----------------------------------------------*/


/* ISR Function --------------------------------------------------------------*/
/**
* @brief :
* @param :
* @retval:
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&hWORK_TIM_Handle);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}


void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&sEV_TIM_Handle);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief :
* @param :
* @retval:
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&hFLASHER_COM_Handle);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}


/**
* @brief :
* @param :
* @retval:
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&hLCD_COM_Handle);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief :
* @param :
* @retval:
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&hRS485_COM_Handle);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}


/**
* @brief :
* @param :
* @retval:
*/


byte chr=0;
void UART4_IRQHandler(void)
{
    
  /* USER CODE BEGIN UART4_IRQn 0 */
    HAL_UART_IRQHandler(&hUSB_COM_Handle);
  // BSP_UART_GetIT_Byte(3,&chr);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}



/**
* @brief :
* @param :
* @retval:
*/


/* Callback Function --------------------------------------------------------------*/
/**
* @brief :
* @param :
* @retval:
*/

/*
        (++) ADC conversion by polling:
          (+++) Activate the ADC peripheral and start conversions
                using function HAL_ADC_Start()
          (+++) Wait for ADC conversion completion 
                using function HAL_ADC_PollForConversion()
                (or for injected group: HAL_ADCEx_InjectedPollForConversion() )
          (+++) Retrieve conversion results 
                using function HAL_ADC_GetValue()
                (or for injected group: HAL_ADCEx_InjectedGetValue() )
          (+++) Stop conversion and disable the ADC peripheral 
                using function HAL_ADC_Stop()
*/
uint32_t adc_val=0;
uint32_t check=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
  //  if(state!=stBoot)
   // bath_polling();
    if(state==stStby||state==stReady||state==stPause||state==stPauseLcd){
    //timer_mk_int();
    //ADC1_INT(); 
    }
    //tm_execute();
   // main_root(); 
    
  //  BSP_UART_PutIT_Byte(3,(byte*)&adc_val,sizeof(adc_val));
    
 
 
    //if(g_dwFpCallBack_Bank[nWORK_TIME_CALLBACK] != 0 )
    //{
     // ((void(*)(void))g_dwFpCallBack_Bank[nWORK_TIME_CALLBACK])();
      
    //}    
  }else if(htim->Instance == TIM3)
  {
     //k =~k;
     //BSP_IndcatorLED_OnOff(1,k); 
  }
  
}




/**
* @brief :
* @param :
* @retval: 
*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  
  // USART1 Rx handling  
  if(huart->Instance == hFLASHER_COM_Handle.Instance)
  {
    //usb_rx_int(chr);  
    BSP_UART_GetIT_Byte(0,&chr);
    setRxQueue(QUSB_RX, chr);
//    if(g_dwFpCallBack_Bank[nFLASHER_RX_CALLBACK] != 0 )
//    {
//      ((void(*)(void))g_dwFpCallBack_Bank[nFLASHER_RX_CALLBACK])();
//    }    
  }

  // USART2 Rx handling  
  if(huart->Instance == hLCD_COM_Handle.Instance)
  {
    //led_rx_int(chr);  
    BSP_UART_GetIT_Byte(1,&chr);
    setRxQueue(QLED_RX, chr);
   /* if(g_dwFpCallBack_Bank[nLCD_RX_CALLBACK] != 0 )
    {
      ((void(*)(void))g_dwFpCallBack_Bank[nLCD_RX_CALLBACK])();
    }  */      
  }

  // USART3 Rx handling  
  if(huart->Instance == hRS485_COM_Handle.Instance)
  {
    if(g_dwFpCallBack_Bank[nRS485_RX_CALLBACK] != 0 )
    {
      ((void(*)(void))g_dwFpCallBack_Bank[nRS485_RX_CALLBACK])();
    }        
  }

  // USART4 Rx handling  
  if(huart->Instance == hUSB_COM_Handle.Instance)
  {
    

      
      
    /*if(g_dwFpCallBack_Bank[nUSB_RX_CALLBACK] != 0 )
    {
      ((void(*)(void))g_dwFpCallBack_Bank[nUSB_RX_CALLBACK])();
    } */       
  }
  
}



/**
* @brief :
* @param :
* @retval:
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  // USART1 Rx handling  
  if(huart->Instance == hFLASHER_COM_Handle.Instance)
  {
//    if(g_dwFpCallBack_Bank[nFLASHER_TX_CALLBACK] != 0 )
//    {
//      ((void(*)(void))g_dwFpCallBack_Bank[nFLASHER_TX_CALLBACK])();
//    }    
  }

  // USART2 Rx handling  
  if(huart->Instance == hLCD_COM_Handle.Instance)
  {
    /*
    if(g_dwFpCallBack_Bank[nLCD_TX_CALLBACK] != 0 )
    {
      ((void(*)(void))g_dwFpCallBack_Bank[nLCD_TX_CALLBACK])();
    }   */     
  }

  // USART3 Rx handling  
  if(huart->Instance == hRS485_COM_Handle.Instance)
  {
    if(g_dwFpCallBack_Bank[nRS485_TX_CALLBACK] != 0 )
    {
      ((void(*)(void))g_dwFpCallBack_Bank[nRS485_TX_CALLBACK])();
    }        
  }

  // USART4 Rx handling  
  if(huart->Instance == hUSB_COM_Handle.Instance)
  {
   /* if(g_dwFpCallBack_Bank[nUSB_TX_CALLBACK] != 0 )
    {
      ((void(*)(void))g_dwFpCallBack_Bank[nUSB_TX_CALLBACK])();
    } */       
  }
}

/**
* @brief :
* @param :
* @retval:
*/
byte flags=0;
void HAL_SYSTICK_Callback(void)
{
  g_dwHW_SystickTimer++;
  if(state==stBoot||state==stStby||state==stReady||state==stPause||state==stPauseLcd||state==stPrepare||state==stPcEng){
  timer_eve();
  timer_mk_int();
  }
  beep_cnt_polling();
 // if(state!=stBoot)
    bath_polling();
   //give_event(eventSystickTimer,0);
}
/**
* @brief :
* @param :
* @retval:
*/


/* Private function  ---------------------------------------------------------*/

/**
* @brief :
* @param :
* @retval:
*/
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    /**NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
    */
  __HAL_AFIO_REMAP_SWJ_ENABLE();
  //__HAL_AFIO_REMAP_SWJ_NOJTAG();
  //__HAL_AFIO_REMAP_SWJ_NOJTAG

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}


/**
* @brief :
* @param :
* @retval:
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN10 
    */
    GPIO_InitStruct.Pin = THERM_ADC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(THERM_ADC_GPIO_Port, &GPIO_InitStruct);


  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }

}



/**
* @brief :
* @param :
* @retval:
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = I2C_SCL_EEPROM_Pin|I2C_SDA_EEPROM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }

}




/**
* @brief :
* @param :
* @retval:
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }else if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}



/**
* @brief :
* @param :
* @retval:
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();
  
    /**UART4 GPIO Configuration    
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX 
    */
   /*
    GPIO_InitStruct.Pin = USB_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(USB_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USB_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_RX_GPIO_Port, &GPIO_InitStruct);
*/
    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(UART4_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = USB_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(USB_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USB_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_RX_GPIO_Port, &GPIO_InitStruct);
    
    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART1_IRQn,2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX 
    */
    
    GPIO_InitStruct.Pin = LCD_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LCD_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LCD_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LCD_RX_GPIO_Port, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_USART2_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
 /*   GPIO_InitStruct.Pin = RS485_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RS485_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RS485_RX_GPIO_Port, &GPIO_InitStruct);
*/
    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }

}

//==============================================================================
// Init Code

/**
* @brief :
* @param :
* @retval:
*/
void HW_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PUMP6_IN2_CCW_Pin|PUMP5_PWM_Pin|PUMP5_IN1_CW_Pin|PUMP5_IN2_CCW_Pin 
                          |PUMP1_PWM_Pin|PUMP1_IN1_CW_Pin|PUMP1_IN2_CCW_Pin|BATH_PIN_ONOFF_Pin 
                          |M0_SW_Pin|PUMP8_IN1_CW_Pin|PUMP8_IN2_CCW_Pin|PUMP6_PWM_Pin 
                          |PUMP6_IN1_CW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FAN_ONOFF_Pin|RESERVE_ONOFF_Pin|LED_POWER_Pin|LED_ASPDSP_Pin 
                          |LED_INCUBATION_Pin|LED_DRY_Pin|PUMP8_PWM_Pin|M2_STB_RST_Pin 
                          |M2_SW_Pin|RS4_Pin|RS5_Pin|M2_STCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOA, PUMP4_PWM_Pin|PUMP4_IN1_CW_Pin|PUMP4_IN2_CCW_Pin|PUMP3_PWM_Pin 
                          |PUMP3_IN1_CW_Pin|PUMP3_IN2_CCW_Pin|PUMP2_PWM_Pin|PUMP2_IN1_CW_Pin 
                          |BUZZER_Pin|PUMP7_PWM_Pin|RS0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PUMP2_IN2_CCW_Pin|HEATER_ONOFF_Pin|M0_CS_Pin|M1_CS_Pin 
                          |M2_CS_Pin|RS1_Pin|RS2_Pin|RS3_Pin 
                          |PUMP7_IN1_CW_Pin|PUMP7_IN2_CCW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PUMP10_IN2_CCW_GPIO_Pin|PUMP10_PWM_Pin|RS7_Pin|M0_STB_RST_Pin 
                          |LED_MCU_CHECK_Pin|M0_STCK_Pin|PUMP9_IN1_CW_Pin|PUMP10_IN1_CW_GPIO_Pin, GPIO_PIN_RESET);
  
   /*Configure GPIO pins : PUMP6_IN2_CCW_Pin PUMP5_PWM_Pin PUMP5_IN1_CW_Pin PUMP5_IN2_CCW_Pin 
                           PUMP1_PWM_Pin PUMP1_IN1_CW_Pin PUMP1_IN2_CCW_Pin BATH_PIN_ONOFF_Pin 
                           M0_SW_Pin PUMP8_IN1_CW_Pin PUMP8_IN2_CCW_Pin PUMP6_PWM_Pin 
                           PUMP6_IN1_CW_Pin */
  GPIO_InitStruct.Pin = PUMP6_IN2_CCW_Pin|PUMP5_PWM_Pin|PUMP5_IN1_CW_Pin|PUMP5_IN2_CCW_Pin 
                          |PUMP1_PWM_Pin|PUMP1_IN1_CW_Pin|PUMP1_IN2_CCW_Pin|BATH_PIN_ONOFF_Pin 
                          |M0_SW_Pin|PUMP8_IN1_CW_Pin|PUMP8_IN2_CCW_Pin|PUMP6_PWM_Pin 
                          |PUMP6_IN1_CW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RESERVE_IN_Pin M2_BUSY_SYNC_Pin M2_FLAG_Pin */
  GPIO_InitStruct.Pin = RESERVE_IN_Pin|M2_BUSY_SYNC_Pin|M2_FLAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN_ONOFF_Pin RESERVE_ONOFF_Pin LED_POWER_Pin LED_ASPDSP_Pin 
                           LED_INCUBATION_Pin LED_DRY_Pin PUMP8_PWM_Pin M2_STB_RST_Pin 
                           M2_SW_Pin RS4_Pin RS5_Pin M2_STCK_Pin */
  GPIO_InitStruct.Pin = FAN_ONOFF_Pin|RESERVE_ONOFF_Pin|LED_POWER_Pin|LED_ASPDSP_Pin 
                          |LED_INCUBATION_Pin|LED_DRY_Pin|PUMP8_PWM_Pin|M2_STB_RST_Pin 
                          |M2_SW_Pin|RS4_Pin|RS5_Pin|M2_STCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PUMP4_PWM_Pin PUMP4_IN1_CW_Pin PUMP4_IN2_CCW_Pin PUMP3_PWM_Pin 
                           PUMP3_IN1_CW_Pin PUMP3_IN2_CCW_Pin PUMP2_PWM_Pin PUMP2_IN1_CW_Pin 
                           BUZZER_Pin PUMP7_PWM_Pin RS0_Pin */
  GPIO_InitStruct.Pin = PUMP4_PWM_Pin|PUMP4_IN1_CW_Pin|PUMP4_IN2_CCW_Pin|PUMP3_PWM_Pin 
                          |PUMP3_IN1_CW_Pin|PUMP3_IN2_CCW_Pin|PUMP2_PWM_Pin|PUMP2_IN1_CW_Pin 
                          |BUZZER_Pin|PUMP7_PWM_Pin|RS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PUMP2_IN2_CCW_Pin HEATER_ONOFF_Pin M0_CS_Pin M1_CS_Pin 
                           M2_CS_Pin RS1_Pin RS2_Pin RS3_Pin 
                           PUMP7_IN1_CW_Pin PUMP7_IN2_CCW_Pin */
  GPIO_InitStruct.Pin = PUMP2_IN2_CCW_Pin|HEATER_ONOFF_Pin|M0_CS_Pin|M1_CS_Pin 
                          |M2_CS_Pin|RS1_Pin|RS2_Pin|RS3_Pin 
                          |PUMP7_IN1_CW_Pin|PUMP7_IN2_CCW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BATH_PIN_1_Pin BATH_PIN_2_Pin M0_BUSY_SYNC_Pin */
  GPIO_InitStruct.Pin = BATH_PIN_1_Pin|BATH_PIN_2_Pin|M0_BUSY_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : M1_STCK_Pin RS6_Pin RS7_Pin M0_STB_RST_Pin 
                           LED_MCU_CHECK_Pin M0_STCK_Pin M1_STB_RST_Pin M1_SW_Pin */
  GPIO_InitStruct.Pin = PUMP10_IN2_CCW_GPIO_Pin|PUMP10_PWM_Pin|RS7_Pin|M0_STB_RST_Pin|M1_HOME_Pin 
                          |LED_MCU_CHECK_Pin|M0_STCK_Pin|PUMP9_IN1_CW_Pin|PUMP10_IN1_CW_GPIO_Pin|PUMP9_IN2_CCW_Pin|PUMP9_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : M2_HOME_Pin M1_HOME_Pin M0_HOME_Pin M0_FLAG_Pin 
                           M1_FLAG_Pin M1_BUSY_SYNC_Pin */
  GPIO_InitStruct.Pin = M2_HOME_Pin|M0_HOME_Pin|M0_FLAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}


/**
* @brief : ADC1 init function 
* @param :
* @retval:
*/
ADC_ChannelConfTypeDef sConfig;
void HW_ADC_Init(void)
{

 

    /**Common config 
    */
  hTHERM_ADC_Handle.Instance                    = ADC1;
  hTHERM_ADC_Handle.Init.ScanConvMode           = ADC_SCAN_DISABLE;
  hTHERM_ADC_Handle.Init.ContinuousConvMode     = DISABLE;
  hTHERM_ADC_Handle.Init.DiscontinuousConvMode  = DISABLE;
  hTHERM_ADC_Handle.Init.ExternalTrigConv       = ADC_SOFTWARE_START;
  hTHERM_ADC_Handle.Init.DataAlign              = ADC_DATAALIGN_RIGHT;
  hTHERM_ADC_Handle.Init.NbrOfConversion        = 1;
  
  if (HAL_ADC_Init(&hTHERM_ADC_Handle) != HAL_OK)
  {
    BSP_System_Error();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel                               = ADC_CHANNEL_10;
  sConfig.Rank                                  = 1;
  sConfig.SamplingTime                          = ADC_SAMPLETIME_41CYCLES_5;
  
  if (HAL_ADC_ConfigChannel(&hTHERM_ADC_Handle, &sConfig) != HAL_OK)
  {
    BSP_System_Error();
  }
  
  dwHsSW_Delay_us(10000);
 // HAL_ADCEx_Calibration_Start(&hTHERM_ADC_Handle);
  dwHsSW_Delay_us(10000);
  

}


/**
* @brief : I2C1 init function
* @param :
* @retval:
*/
void HW_I2C_Init(void)
{

  hEEPROM_I2C_Handle.Instance                   = I2C1;
  hEEPROM_I2C_Handle.Init.ClockSpeed            = 110000;
  hEEPROM_I2C_Handle.Init.DutyCycle             = I2C_DUTYCYCLE_2;
  hEEPROM_I2C_Handle.Init.OwnAddress1           = 0;
  hEEPROM_I2C_Handle.Init.AddressingMode        = I2C_ADDRESSINGMODE_7BIT;
  hEEPROM_I2C_Handle.Init.DualAddressMode       = I2C_DUALADDRESS_DISABLE;
  hEEPROM_I2C_Handle.Init.OwnAddress2           = 0;
  hEEPROM_I2C_Handle.Init.GeneralCallMode       = I2C_GENERALCALL_DISABLE;
  hEEPROM_I2C_Handle.Init.NoStretchMode         = I2C_NOSTRETCH_DISABLE;
  
  if (HAL_I2C_Init(&hEEPROM_I2C_Handle) != HAL_OK)
  {
    BSP_System_Error();
  }
  //HAL_I2C_MspInit(&hEEPROM_I2C_Handle);

}

/**
* @brief : TIM1, TIM1 init function
* @param :
* @retval:
*/
 
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_ClockConfigTypeDef sClockSourceConfig;
void HW_TIM_Init(void)
{
  //TIM1


  hSERVO_TIM_Handle.Instance = TIM1;
  hSERVO_TIM_Handle.Init.Prescaler = 71;
  hSERVO_TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  hSERVO_TIM_Handle.Init.Period = 19999;
  hSERVO_TIM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  hSERVO_TIM_Handle.Init.RepetitionCounter = 0;

  
  if (HAL_TIM_PWM_Init(&hSERVO_TIM_Handle) != HAL_OK)
  {
    BSP_System_Error();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  
  if (HAL_TIMEx_MasterConfigSynchronization(&hSERVO_TIM_Handle, &sMasterConfig) != HAL_OK)
  {
    BSP_System_Error();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 17800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&hSERVO_TIM_Handle, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    BSP_System_Error();
  }

  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&hSERVO_TIM_Handle, &sBreakDeadTimeConfig) != HAL_OK)
  {
    BSP_System_Error();
  }

  HAL_TIM_MspPostInit(&hSERVO_TIM_Handle);

  //HAL_Delay(3000);
  //HAL_TIM_OC_Start_IT(&hSERVO_TIM_Handle, TIM_CHANNEL_4);

  //TIM2
  hWORK_TIM_Handle.Instance = TIM2;
  hWORK_TIM_Handle.Init.Prescaler = 72;
  hWORK_TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  hWORK_TIM_Handle.Init.Period = 999;
  hWORK_TIM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  
  if (HAL_TIM_Base_Init(&hWORK_TIM_Handle) != HAL_OK)
  {
    BSP_System_Error();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  
  if (HAL_TIM_ConfigClockSource(&hWORK_TIM_Handle, &sClockSourceConfig) != HAL_OK)
  {
    BSP_System_Error();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&hWORK_TIM_Handle, &sMasterConfig) != HAL_OK)
  {
    BSP_System_Error();
  }
  
 // HAL_TIM_Base_Start_IT(&hWORK_TIM_Handle);
  
  
  
   //TIM3
  sEV_TIM_Handle.Instance = TIM3;
  sEV_TIM_Handle.Init.Prescaler = 72;
  sEV_TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  sEV_TIM_Handle.Init.Period = 1;
  sEV_TIM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  
  if (HAL_TIM_Base_Init(&sEV_TIM_Handle) != HAL_OK)
  {
    BSP_System_Error();
  }

  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  
  if (HAL_TIM_ConfigClockSource(&sEV_TIM_Handle, &sClockSourceConfig) != HAL_OK)
  {
    BSP_System_Error();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&sEV_TIM_Handle, &sMasterConfig) != HAL_OK)
  {
    BSP_System_Error();
  }
  

  
 //  HAL_TIM_Base_Start_IT(&sEV_TIM_Handle);

}
void Servo_MT_init()
{
    HAL_TIM_PWM_Start_IT(&hSERVO_TIM_Handle, TIM_CHANNEL_4);
}

void Servo_MT_stop()
{
  HAL_TIM_PWM_Stop(&hSERVO_TIM_Handle, TIM_CHANNEL_4);
}

   // TIM_OC_InitTypeDef sConfigOC;
void servo_mv(unsigned int pos) //SERVO MOVE(without overcurrent detecting)
{
 // timer_intrrupt_stop();
//  HAL_TIM_PWM_Start_IT(&hSERVO_TIM_Handle, TIM_CHANNEL_4);
  TIM1->CCR4=(pos);
 // HAL_TIM_PWM_Stop(&hSERVO_TIM_Handle, TIM_CHANNEL_4);
/*
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pos;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&hSERVO_TIM_Handle, &sConfigOC, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start_IT(&hSERVO_TIM_Handle, TIM_CHANNEL_4);
   set_timer_(eventAspTimeOut,1000,0); 
  */
  // __HAL_TIM_SetCompare(&hSERVO_TIM_Handle, TIM_CHANNEL_4, pos);
}

void Time2_It_Stop()
{
  HAL_TIM_Base_Stop_IT(&hWORK_TIM_Handle);
}

void Time2_It_Start()
{
  HAL_TIM_Base_Start_IT(&hWORK_TIM_Handle);
}
/**
* @brief : USART1 init function 
* @param :
* @retval:
*/
//unsigned char usb_test[50]={0,};
//byte usb_receive_bufs[RECEIVE_BUF_LENGTH];
void HW_UART_Init(void)
{
  //USART1
  hFLASHER_COM_Handle.Instance                  = USART1;
  hFLASHER_COM_Handle.Init.BaudRate             = 57600;
  hFLASHER_COM_Handle.Init.WordLength           = UART_WORDLENGTH_8B;
  hFLASHER_COM_Handle.Init.StopBits             = UART_STOPBITS_1;
  hFLASHER_COM_Handle.Init.Parity               = UART_PARITY_NONE;
  hFLASHER_COM_Handle.Init.Mode                 = UART_MODE_TX_RX;
  hFLASHER_COM_Handle.Init.HwFlowCtl            = UART_HWCONTROL_NONE;
  hFLASHER_COM_Handle.Init.OverSampling         = UART_OVERSAMPLING_16;
  
  if (HAL_UART_Init(&hFLASHER_COM_Handle) != HAL_OK)
  {
    BSP_System_Error();
  }
  BSP_UART_GetIT_Byte(0,&chr);
  //USART2
  hLCD_COM_Handle.Instance = USART2;
  hLCD_COM_Handle.Init.BaudRate = 115200;
  hLCD_COM_Handle.Init.WordLength = UART_WORDLENGTH_8B;
  hLCD_COM_Handle.Init.StopBits = UART_STOPBITS_1;
  hLCD_COM_Handle.Init.Parity = UART_PARITY_NONE;
  hLCD_COM_Handle.Init.Mode = UART_MODE_TX_RX;
  hLCD_COM_Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hLCD_COM_Handle.Init.OverSampling = UART_OVERSAMPLING_16;
  
  if (HAL_UART_Init(&hLCD_COM_Handle) != HAL_OK)
  {
    BSP_System_Error();
  }
  BSP_UART_GetIT_Byte(1,&chr);
  //USART3
  hRS485_COM_Handle.Instance = USART3;
  hRS485_COM_Handle.Init.BaudRate = 115200;
  hRS485_COM_Handle.Init.WordLength = UART_WORDLENGTH_8B;
  hRS485_COM_Handle.Init.StopBits = UART_STOPBITS_1;
  hRS485_COM_Handle.Init.Parity = UART_PARITY_NONE;
  hRS485_COM_Handle.Init.Mode = UART_MODE_TX_RX;
  hRS485_COM_Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hRS485_COM_Handle.Init.OverSampling = UART_OVERSAMPLING_16;
  
  if (HAL_UART_Init(&hRS485_COM_Handle) != HAL_OK)
  {
    BSP_System_Error();
  }

  //UART4
  hUSB_COM_Handle.Instance = UART4;
  hUSB_COM_Handle.Init.BaudRate = 57600;//115200;
  hUSB_COM_Handle.Init.WordLength = UART_WORDLENGTH_8B;
  hUSB_COM_Handle.Init.StopBits = UART_STOPBITS_1;
  hUSB_COM_Handle.Init.Parity = UART_PARITY_NONE;
  hUSB_COM_Handle.Init.Mode = UART_MODE_TX_RX;
  hUSB_COM_Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hUSB_COM_Handle.Init.OverSampling = UART_OVERSAMPLING_16;
  
  
  
  if (HAL_UART_Init(&hUSB_COM_Handle) != HAL_OK)
  {
    BSP_System_Error();
  }
  // BSP_UART_GetIT_Byte(3,&chr);

}

/**
* @brief :
* @param :
* @retval:
*/


void HW_SPI2_Init(void)
{
  smt.Instance = SPI2;
  smt.Init.Mode = SPI_MODE_MASTER;
  smt.Init.Direction = SPI_DIRECTION_2LINES;
  smt.Init.DataSize = SPI_DATASIZE_8BIT;
  smt.Init.CLKPolarity = SPI_POLARITY_LOW;
  smt.Init.CLKPhase = SPI_PHASE_1EDGE;
  smt.Init.NSS = SPI_NSS_SOFT;
  smt.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  smt.Init.FirstBit = SPI_FIRSTBIT_MSB;
  smt.Init.TIMode = SPI_TIMODE_DISABLE;
  smt.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  smt.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&smt) != HAL_OK)
  {
    BSP_System_Error();
  }
  set_spi_handle(&smt); 
  
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
void BSP_MCU_Init(uint32_t *p_FP)
{
#if 0
  uint32_t dwCnt;
  
  for(dwCnt =0; ;dwCnt++)
  {
    if(p_FP[dwCnt] == 0) break;
    g_dwFpCallBack_Bank[dwCnt] = p_FP[dwCnt];
  }
#endif
  
  //HW Init
  HW_GPIO_Init();
  HW_ADC_Init();
  HW_I2C_Init();
  HW_UART_Init();
  HW_TIM_Init();
  HW_SPI2_Init();

 // HW_SPI1_Init();
  
  //IO Base Settin
  
  
  //BSP_MCU_Check_LEDOnOff(1);
  BSP_ETC_OnOff_Device(FAN,FAN_OFF);
  BSP_ETC_OnOff_Device(HEATER,HEATER_OFF);
  

}


/**
* @brief :
* @param :
* @retval:
*/
uint32_t BSP_Version(void)
{
  return (uint32_t)(__HS_BSP_VERSION);
}

/**
* @brief :
* @param :
* @retval:
*/
void BSP_MCU_Check_LEDOnOff(uint32_t dwOn_nOff)
{
  if(dwOn_nOff == 1) //On
  {
    BSP_IndcatorLED_OnOff(0, 0);
  }
  else // Off
  {
    BSP_IndcatorLED_OnOff(0, 1);
  }
}

/**
* @brief :
* @param :
* @retval:
*/
void BSP_System_Error(void)
{
  uint32_t dwFlag = 1;
  
  while(dwFlag)
  {
    //BSP_IndcatorLED_OnOff(0, 0);
    BSP_MCU_Check_LEDOnOff(1);
    dwHsSW_Delay_ms(1000);
    //BSP_IndcatorLED_OnOff(0, 1);
    BSP_MCU_Check_LEDOnOff(0);
    dwHsSW_Delay_ms(1000);
  }
}


/**
* @brief :
* @param :
* @retval:

MCU_CHECK :Dev_num(0), Low(0)-LED On

*/
void BSP_IndcatorLED_OnOff(uint32_t dwDev_num, uint32_t dwPinState)
{
  if(dwPinState >  1) dwPinState = 1;
  
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_IndcatorLED[dwDev_num], (uint16_t)g_GPIO_Pin_IndcatorLED[dwDev_num],(GPIO_PinState)dwPinState);
}


/**
* @brief :
* @param :
* @retval:
*/
void BSP_TB6617_Enable(uint32_t dwDev_num)
{  
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_TB6617FNG_PWM[dwDev_num], (uint16_t)g_GPIO_Pin_TB6617FNG_PWM[dwDev_num], GPIO_PIN_SET);
}

/**
* @brief :
* @param :
* @retval:
*/
void BSP_TB6617_Diasble(uint32_t dwDev_num)
{  
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_TB6617FNG_PWM[dwDev_num], (uint16_t)g_GPIO_Pin_TB6617FNG_PWM[dwDev_num], GPIO_PIN_RESET);
}

/**
* @brief :
* @param :
* @retval:
*/
void BSP_TB6617_Break(uint32_t dwDev_num)
{
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_TB6617FNG_CW[dwDev_num], (uint16_t)g_GPIO_Pin_TB6617FNG_CW[dwDev_num], GPIO_PIN_SET);
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_TB6617FNG_CCW[dwDev_num], (uint16_t)g_GPIO_Pin_TB6617FNG_CCW[dwDev_num], GPIO_PIN_SET);
}
/**
* @brief :
* @param :
* @retval:
*/
void BSP_TB6617_Stop(uint32_t dwDev_num)
{
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_TB6617FNG_CW[dwDev_num], (uint16_t)g_GPIO_Pin_TB6617FNG_CW[dwDev_num], GPIO_PIN_RESET);
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_TB6617FNG_CCW[dwDev_num], (uint16_t)g_GPIO_Pin_TB6617FNG_CCW[dwDev_num], GPIO_PIN_RESET);
}

/**
* @brief :
* @param :
* @retval:
*/
void BSP_TB6617_CW(uint32_t dwDev_num)
{
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_TB6617FNG_CW[dwDev_num], (uint16_t)g_GPIO_Pin_TB6617FNG_CW[dwDev_num], GPIO_PIN_SET);
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_TB6617FNG_CCW[dwDev_num], (uint16_t)g_GPIO_Pin_TB6617FNG_CCW[dwDev_num], GPIO_PIN_RESET);
}

/**
* @brief :
* @param :
* @retval:
*/
void BSP_TB6617_CCW(uint32_t dwDev_num)
{
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_TB6617FNG_CW[dwDev_num], (uint16_t)g_GPIO_Pin_TB6617FNG_CW[dwDev_num], GPIO_PIN_RESET);
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_TB6617FNG_CCW[dwDev_num], (uint16_t)g_GPIO_Pin_TB6617FNG_CCW[dwDev_num], GPIO_PIN_SET);
}

/**
* @brief :
* @param :
* @retval:
*/
void BSP_DRV8825_StandBy(uint32_t dwDev_num)
{
  /*
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_DIR[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_DIR[dwDev_num], GPIO_PIN_RESET);
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_STEP[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_STEP[dwDev_num], GPIO_PIN_RESET);
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_SLEEP[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_SLEEP[dwDev_num], GPIO_PIN_RESET);
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_RESET[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_RESET[dwDev_num], GPIO_PIN_RESET);
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_DECAY[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_DECAY[dwDev_num], GPIO_PIN_RESET);
  
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_ENABLE[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_ENABLE[dwDev_num], GPIO_PIN_SET);
  
  dwHsSW_Delay_us(500);
  
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_RESET[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_RESET[dwDev_num], GPIO_PIN_SET);
  dwHsSW_Delay_us(2);
  
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_SLEEP[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_SLEEP[dwDev_num], GPIO_PIN_SET);
  dwHsSW_Delay_us(2000);
  
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_ENABLE[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_ENABLE[dwDev_num], GPIO_PIN_RESET);
  dwHsSW_Delay_us(2);
  */
}

/**
* @brief :
* @param :
* @retval:
*/
void BSP_DRV8825_ENABLE(uint32_t dwDev_num, uint32_t dwEable)
{
  //HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_ENABLE[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_ENABLE[dwDev_num], (GPIO_PinState)dwEable);
}
/**
* @brief :
* @param :
* @retval:
*/
void BSP_DRV8825_DIR(uint32_t dwDev_num, uint32_t dwCCW_nCW)
{
  //dwCCW_nCW - 0 CW, 1 CCW
  //HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_DIR[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_DIR[dwDev_num], (GPIO_PinState)dwCCW_nCW);
}
/**
* @brief :
* @param :
* @retval:
*/
void BSP_DRV8825_DECAY(uint32_t dwDev_num, uint32_t dwFast_nLow)
{
  //dwFast_nLow: 0 nLow, 1 Fast Decay mod
 // HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_DECAY[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_DECAY[dwDev_num], (GPIO_PinState)dwFast_nLow);
}

/**
* @brief :
* @param :
* @retval:
*/
void BSP_DRV8825_RESET(uint32_t dwDev_num)
{
//  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_RESET[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_RESET[dwDev_num], GPIO_PIN_RESET);
//  dwHsSW_Delay_us(10);
//  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_RESET[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_RESET[dwDev_num], GPIO_PIN_SET); 
//  dwHsSW_Delay_us(5);
}

/**
* @brief :
* @param :
* @retval:
*/


void BSP_DRV8825_STEP(uint32_t dwDev_num, uint32_t dwPulse_us)
{
  
//    HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_STEP[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_STEP[dwDev_num], GPIO_PIN_SET);
//    dwHsSW_Delay_us(dwPulse_us);
//    HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_DRV8825_STEP[dwDev_num], (uint16_t)g_GPIO_Pin_DRV8825_STEP[dwDev_num], GPIO_PIN_RESET); 
//    dwHsSW_Delay_us(dwPulse_us);
    
}
/**
* @brief :
* @param :
* @retval:
*/
//int32_t BSP_DRV8825_Fault_Check(uint32_t dwDev_num)
//{
//  uint32_t dwPinState = 0;
//  dwPinState = (uint32_t)HAL_GPIO_ReadPin((GPIO_TypeDef*) gp_PORT_DRV8825_FAULT[dwDev_num], (uint16_t) g_GPIO_Pin_DRV8825_FAULT[dwDev_num]);
//  
//  if(dwPinState == 1) dwPinState = 0;
//  else dwPinState = 1;
//  
//  return dwPinState;
//}

/**
* @brief :
* @param :
* @retval:
*/
int32_t BSP_DRV8825_Home_Check(uint32_t dwDev_num)
{
  uint32_t dwPinState = 0;
  dwPinState = (uint32_t)HAL_GPIO_ReadPin((GPIO_TypeDef*) gp_PORT_DRV8825_HOME_SEN[dwDev_num], (uint16_t) g_GPIO_Pin_DRV8825_HOME_SEN[dwDev_num]);
  
  return dwPinState;
}


/**
* @brief :
* @param :
* @retval:
*/
int32_t BSP_AT24_I2C_Write(uint16_t wDevAddr, uint16_t wMemAddr, uint16_t wMemAddrSize, uint8_t *p_uData, uint16_t wSize, uint32_t dwTimeout)
{
  HAL_StatusTypeDef tCheck;
  uint32_t dwCnt = 0;
  
  while(1)
  {
    tCheck = HAL_I2C_Mem_Write( &hEEPROM_I2C_Handle, wDevAddr, wMemAddr, wMemAddrSize, p_uData, wSize, dwTimeout);
    
    if( tCheck == HAL_OK ) return 0;
    else if( tCheck == HAL_BUSY)
    {
      if(dwCnt > 5) return 1;
      else
      {
        dwHsSW_Delay_ms(1);
        dwCnt++;
      }
    }
    else break;
  }
  
  return 1;
}


/**
* @brief :
* @param :
* @retval:
*/
int32_t BSP_AT24_I2C_Read(uint16_t wDevAddr, uint16_t wMemAddr, uint16_t wMemAddrSize, uint8_t *p_uData, uint16_t wSize, uint32_t dwTimeout)
{
  HAL_StatusTypeDef tCheck;
  uint32_t dwCnt = 0;
  
  while(1)
  {
    tCheck = HAL_I2C_Mem_Read( &hEEPROM_I2C_Handle, wDevAddr, wMemAddr, wMemAddrSize, p_uData, wSize, dwTimeout);

    if( tCheck == HAL_OK ) return 0;
    else if( tCheck == HAL_BUSY)
    {
      if(dwCnt > 5) return 1;
      else
      {
        dwHsSW_Delay_ms(1);
        dwCnt++;
      }
    }
    else break;

  }
  
  return 1;
}

/**
* @brief :
* @param :
* @retval:
*/
void BSP_ETC_OnOff_Device(uint32_t dwDev_num, uint32_t dwPinState)
{
  if(dwPinState >  1) dwPinState = 1;
  
  HAL_GPIO_WritePin( (GPIO_TypeDef *)gp_PORT_ETC_ONOFF[dwDev_num], (uint16_t)g_GPIO_Pin_ETC_ONOFF[dwDev_num],(GPIO_PinState)dwPinState); 
}
/**
* @brief :
* @param :
* @retval:
*/
int32_t BSP_ETC_Input_Check(uint32_t dwDev_num)
{
  uint32_t dwPinState = 0;
  dwPinState = (uint32_t)HAL_GPIO_ReadPin((GPIO_TypeDef*) gp_PORT_ETC_INPUT[dwDev_num], (uint16_t) g_GPIO_Pin_ETC_INPUT[dwDev_num]);
  
  return dwPinState;
}

/**
* @brief :
* @param :
* @retval:
*/
void BSP_UART_GetIT_Byte(uint32_t dwDev_num, uint8_t *p_uGetData)
{
  HAL_StatusTypeDef tState;
  UART_HandleTypeDef *p_Uart = (UART_HandleTypeDef*)gp_UART_Handle[dwDev_num];
  uint8_t *p_uData = p_uGetData;
  
  tState = HAL_UART_Receive_IT( p_Uart, p_uData, sizeof(uint8_t));
  if(tState != HAL_OK)
  {
    if(tState == HAL_BUSY)
    {
      p_Uart->State = HAL_UART_STATE_READY;
      __HAL_UNLOCK(p_Uart);
      tState = HAL_UART_Receive_IT( p_Uart, p_uData, sizeof(uint8_t));
      if(tState != HAL_OK) BSP_System_Error(); 
    }
    else
    {
      BSP_System_Error();
    }
  }
  
}


/**
* @brief :
* @param :
* @retval:
*/
int32_t BSP_UART_PutIT_Byte(uint32_t dwDev_num, uint8_t *p_uPutData, uint16_t wLen)
{
  UART_HandleTypeDef *p_Uart = (UART_HandleTypeDef*)gp_UART_Handle[dwDev_num];
  //original
  // if( HAL_UART_GetState(p_Uart) == HAL_UART_STATE_BUSY_TX) return 1;
  // while(HAL_UART_Transmit(p_Uart, p_uPutData, wLen, 1000) != HAL_OK);
  
  //convert
  setUsbTxQueue(dwDev_num, p_uPutData, wLen);

  //while(HAL_UART_Transmit_IT(p_Uart, p_uPutData, wLen) != HAL_OK);
//  if(HAL_UART_Transmit_IT(p_Uart, p_uPutData, wLen) != HAL_OK)
//  {
//   BSP_System_Error();
//  }
  //BSP_UART_GetIT_Byte(3,&chr);
  return 0;
}


/**
* @brief :
* @param :
* @retval:
__IO uint32_t g_dwHW_SystickTimer = 0;
uint32_t g_dwHW_TimeSet[10];
*/
void BSP_SystickTimerSet(uint32_t TimerBank_num)
{
  g_dwHW_TimeSet[TimerBank_num] = g_dwHW_SystickTimer;
}
/**
* @brief :
* @param :
* @retval:
*/
uint32_t BSP_SystickTimerCurrentValue(uint32_t TimerBank_num)
{
 uint32_t dwReturnValue;
 uint32_t dwShotTime;
 
 dwShotTime = g_dwHW_SystickTimer;
 
 if(g_dwHW_TimeSet[TimerBank_num] > dwShotTime)
 {
   dwReturnValue = (0xFFFFFFFF - g_dwHW_TimeSet[TimerBank_num]) + dwShotTime;
 }
 else
 {
   dwReturnValue = dwShotTime - g_dwHW_TimeSet[TimerBank_num];
 }
 
 return dwReturnValue;
}
/**
* @brief :
* @param :
* @retval:
nHW_SYSTEM_TIMER
*/
void BSP_SystemHW_Delay_msec(uint32_t dwWaitTime_msec)
{
  BSP_SystickTimerSet(nHW_SYSTEM_TIMER);
  
  while(1)
  {
    if(g_dwSystemHW_TimerBreak > 0) break;
    if(dwWaitTime_msec < BSP_SystickTimerCurrentValue(nHW_SYSTEM_TIMER) ) break;
  }
  
  g_dwSystemHW_TimerBreak = 0;
}
/**
* @brief :
* @param :
* @retval:
*/
void BSP_SystemHW_DelayBreakerHandler(uint32_t dwSetting)
{
  g_dwSystemHW_TimerBreak = dwSetting;
}
/**
* @brief :
* @param :
* @retval:
*/
uint32_t BSP_SystickTimCountGet(void)
{
  return g_dwHW_SystickTimer;
}
/**
* @brief :
* @param :
* @retval:
*/
void USB_IT_Disable(void)
{
 __HAL_UART_DISABLE_IT(&hFLASHER_COM_Handle,UART_IT_RXNE) ;
}
/**
* @brief :
* @param :
* @retval:
*/
void USB_IT_Enable(void)
{
 __HAL_UART_ENABLE_IT(&hFLASHER_COM_Handle,UART_IT_RXNE) ;
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
