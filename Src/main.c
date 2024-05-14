/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
const uint32_t g_dwFpCallbackArry[nCALLBACK_n];
CQueue mQueue;
QData qData;
int qi;
uint32_t qSize[QID_COUNT];
byte qRx[QID_COUNT];
bool swapTemp = true;
uint32_t temp_count = 100;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/*
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
*/
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void SystemClock_Config(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);

// void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void setUsbTxQueue(uint32_t dwDev_num, byte *data, uint16_t length)
{
    mQueue.enqueueTX(QUSB_TX, dwDev_num, data, length);
}

int txProc()
{    
    if (mQueue.isEmpty(QUSB_TX) && mQueue.front(QUSB_TX, &qData))
        return 2;//데이터 없음
     
    UART_HandleTypeDef *p_Uart = (UART_HandleTypeDef*)gp_UART_Handle[qData.dwDev_num];
    if (HAL_UART_GetState(p_Uart) == HAL_UART_STATE_BUSY_TX)
    {
        return 1; // 전송 중인 경우 오류 반환
    }

    // UART 전송을 비동기적으로 시작
    if (HAL_UART_Transmit_IT(p_Uart, qData.buffer, qData.length) != HAL_OK)
    {
        // 오류 처리
        BSP_System_Error();
        return -1; // 오류 발생 시 -1 반환
    }

    mQueue.next(QUSB_TX);//다음 데이터로 이동
    return 0; // 성공적으로 전송 시작 시 0 반환
}

void setRxQueue(QID id, byte data)
{
    mQueue.enqueue(id, data);
}

int rxProc(QID id)
{
    qSize[id] = mQueue.size(id);
    
    for(qi = 0; qi < qSize[id]; qi++)
    {
        if(mQueue.dequeue(id, &qRx[id]))
        {
            switch (id)
            {
                case QUSB_RX: usb_rx_int(qRx[id]); break;
                case QLED_RX: led_rx_int(qRx[id]); break;
                default: break;
            }
        }
    }
}

void tempProc()
{
    if(++temp_count < 100)
        return;

    if(swapTemp)
        readAirTemp();
    else
        readTrayTemp();

    swapTemp = !swapTemp;
    temp_count = 0;
}
/* USER CODE END 0 */

int main(void)
{

    /* USER CODE BEGIN 1 */
    mQueue = createQueue();    
    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    // SystemClock_Config();
    MX_TIM2_Init();
    MX_SPI2_Init();

    /* USER CODE BEGIN 2 */
    // beep(0,100,1);
    state = stBoot;
    HAL_GPIO_WritePin(GPIOB, M1_CS_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED_POWER_GPIO_Port, LED_POWER_Pin, GPIO_PIN_RESET);
    BSP_MCU_Init((uint32_t *)g_dwFpCallbackArry);
    BSP_IndcatorLED_OnOff(1, 1);

    tmchan_init();
    st_init();
    // BSP_IndcatorLED_OnOff(1,1); //usbOnOff
    AirFan_Off;

    // X_AIS_Home();
    // Sk_Home();
    //  Pump_Multi_Run(0x03,PUMP_CW);
    eeprom_init();
    Servo_MT_init();

    // led_read();
    // HAL_TIM_PWM_Start_IT(&hSERVO_TIM_Handle, TIM_CHANNEL_4);
    set_timer_(eventSpuOn, 3000, 0);
    // Stmt_AbsMove(X_AIS_MT,X_AIS_SPD,8000);
    // Pump_Single_Run(0x03,usb_data_buf[1],0);
    //   Pump_Single_Run(1,PUMP_CW,3000);
    // beep(0,500,3);
    // Pump_Single_Run(PUMP_ASP1+1,PUMP_CCW,3000);

    // servo_mv(3750);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    // BSP_UART_GetIT_Byte(3,uart_test);
    // Stmt_Homming(X_AIS_MT,100);
    // Stm_Homming(2,500);
    // Stmt_AbsMove(0, 50, 80000);    
    while (1)
    {
        /* USER CODE END WHILE */
        //tempProc();

        rxProc(QUSB_RX);
        rxProc(QLED_RX);
        
        main_root();

        txProc();
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

void main_root()
{
    tm_execute();
    event event = get_event();
    event = execute_control(event);
    motor_control(event);
    pump_control(event);
    sq_control(event);
    pp_cal_control(event);
    execute_usb_rs232();
    event_dispatch();

    switch (event)
    {
    }
}

void sub_root()
{
    event event = get_event();
    event = execute_control(event);
    execute_usb_rs232();
}
/** System Clock Configuration
 */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
#if 0
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{
#if 0
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{
#if 0
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{
#if 0
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
#endif
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{
#if 0
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
#if 0
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{
#if 0
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{
#if 0
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
  HAL_GPIO_WritePin(GPIOD, M1_STCK_Pin|RS6_Pin|RS7_Pin|M0_STB_RST_Pin 
                          |LED_MCU_CHECK_Pin|M0_STCK_Pin|M1_STB_RST_Pin|M1_SW_Pin, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Pin = M1_STCK_Pin|RS6_Pin|RS7_Pin|M0_STB_RST_Pin|M1_HOME_Pin 
                          |LED_MCU_CHECK_Pin|M0_STCK_Pin|M1_STB_RST_Pin|M1_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : M2_HOME_Pin M1_HOME_Pin M0_HOME_Pin M0_FLAG_Pin 
                           M1_FLAG_Pin M1_BUSY_SYNC_Pin */
  GPIO_InitStruct.Pin = M2_HOME_Pin|M0_HOME_Pin|M0_FLAG_Pin 
                          |M1_FLAG_Pin|M1_BUSY_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
#endif
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    uint32_t dwFlag = 1;
    /* User can add his own implementation to report the HAL error return state */
    while (dwFlag)
    {
    }
    /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
