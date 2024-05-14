/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define PUMP6_IN2_CCW_Pin GPIO_PIN_2
#define PUMP6_IN2_CCW_GPIO_Port GPIOE
#define PUMP5_PWM_Pin GPIO_PIN_3
#define PUMP5_PWM_GPIO_Port GPIOE
#define PUMP5_IN1_CW_Pin GPIO_PIN_4
#define PUMP5_IN1_CW_GPIO_Port GPIOE
#define PUMP5_IN2_CCW_Pin GPIO_PIN_5
#define PUMP5_IN2_CCW_GPIO_Port GPIOE
#define PUMP1_PWM_Pin GPIO_PIN_6
#define PUMP1_PWM_GPIO_Port GPIOE
#define RESERVE_IN_Pin GPIO_PIN_13
#define RESERVE_IN_GPIO_Port GPIOC
#define FAN_ONOFF_Pin GPIO_PIN_14
#define FAN_ONOFF_GPIO_Port GPIOC
#define RESERVE_ONOFF_Pin GPIO_PIN_15
#define RESERVE_ONOFF_GPIO_Port GPIOC
#define THERM_ADC_Pin GPIO_PIN_0
#define THERM_ADC_GPIO_Port GPIOC
#define LED_POWER_Pin GPIO_PIN_1
#define LED_POWER_GPIO_Port GPIOC
#define LED_ASPDSP_Pin GPIO_PIN_2
#define LED_ASPDSP_GPIO_Port GPIOC
#define LED_INCUBATION_Pin GPIO_PIN_3
#define LED_INCUBATION_GPIO_Port GPIOC
#define PUMP4_PWM_Pin GPIO_PIN_0
#define PUMP4_PWM_GPIO_Port GPIOA
#define PUMP4_IN1_CW_Pin GPIO_PIN_1
#define PUMP4_IN1_CW_GPIO_Port GPIOA
#define PUMP4_IN2_CCW_Pin GPIO_PIN_2
#define PUMP4_IN2_CCW_GPIO_Port GPIOA
#define PUMP3_PWM_Pin GPIO_PIN_3
#define PUMP3_PWM_GPIO_Port GPIOA
#define PUMP3_IN1_CW_Pin GPIO_PIN_4
#define PUMP3_IN1_CW_GPIO_Port GPIOA
#define PUMP3_IN2_CCW_Pin GPIO_PIN_5
#define PUMP3_IN2_CCW_GPIO_Port GPIOA
#define PUMP2_PWM_Pin GPIO_PIN_6
#define PUMP2_PWM_GPIO_Port GPIOA
#define PUMP2_IN1_CW_Pin GPIO_PIN_7
#define PUMP2_IN1_CW_GPIO_Port GPIOA
#define LED_DRY_Pin GPIO_PIN_4
#define LED_DRY_GPIO_Port GPIOC
#define PUMP8_PWM_Pin GPIO_PIN_5
#define PUMP8_PWM_GPIO_Port GPIOC
#define PUMP2_IN2_CCW_Pin GPIO_PIN_0
#define PUMP2_IN2_CCW_GPIO_Port GPIOB
#define HEATER_ONOFF_Pin GPIO_PIN_1
#define HEATER_ONOFF_GPIO_Port GPIOB
#define PUMP1_IN1_CW_Pin GPIO_PIN_7
#define PUMP1_IN1_CW_GPIO_Port GPIOE
#define PUMP1_IN2_CCW_Pin GPIO_PIN_8
#define PUMP1_IN2_CCW_GPIO_Port GPIOE
#define BATH_PIN_1_Pin GPIO_PIN_9
#define BATH_PIN_1_GPIO_Port GPIOE
#define BATH_PIN_2_Pin GPIO_PIN_10
#define BATH_PIN_2_GPIO_Port GPIOE
#define BATH_PIN_ONOFF_Pin GPIO_PIN_11
#define BATH_PIN_ONOFF_GPIO_Port GPIOE
#define M0_BUSY_SYNC_Pin GPIO_PIN_12
#define M0_BUSY_SYNC_GPIO_Port GPIOE
#define M0_SW_Pin GPIO_PIN_13
#define M0_SW_GPIO_Port GPIOE
#define PUMP8_IN1_CW_Pin GPIO_PIN_14
#define PUMP8_IN1_CW_GPIO_Port GPIOE
#define PUMP8_IN2_CCW_Pin GPIO_PIN_15
#define PUMP8_IN2_CCW_GPIO_Port GPIOE
#define M0_CS_Pin GPIO_PIN_10
#define M0_CS_GPIO_Port GPIOB
#define M1_CS_Pin GPIO_PIN_11
#define M1_CS_GPIO_Port GPIOB
#define M2_CS_Pin GPIO_PIN_12
#define M2_CS_GPIO_Port GPIOB
#define M_CK_Pin GPIO_PIN_13
#define M_CK_GPIO_Port GPIOB
#define M_SDO_Pin GPIO_PIN_14
#define M_SDO_GPIO_Port GPIOB
#define M_SDI_Pin GPIO_PIN_15
#define M_SDI_GPIO_Port GPIOB
#define PUMP10_IN2_CCW_GPIO_Pin GPIO_PIN_8
#define PUMP10_IN2_CCW_GPIO_Port GPIOD
#define PUMP10_PWM_Pin GPIO_PIN_9
#define PUMP10_PWM_GPIO_Port GPIOD
#define RS7_Pin GPIO_PIN_10
#define RS7_GPIO_Port GPIOD
#define M2_HOME_Pin GPIO_PIN_11
#define M2_HOME_GPIO_Port GPIOD
#define M1_HOME_Pin GPIO_PIN_12
#define M1_HOME_GPIO_Port GPIOD
#define M0_HOME_Pin GPIO_PIN_13
#define M0_HOME_GPIO_Port GPIOD
#define M0_STB_RST_Pin GPIO_PIN_14
#define M0_STB_RST_GPIO_Port GPIOD
#define M0_FLAG_Pin GPIO_PIN_15
#define M0_FLAG_GPIO_Port GPIOD
#define M2_STB_RST_Pin GPIO_PIN_6
#define M2_STB_RST_GPIO_Port GPIOC
#define M2_BUSY_SYNC_Pin GPIO_PIN_7
#define M2_BUSY_SYNC_GPIO_Port GPIOC
#define M2_FLAG_Pin GPIO_PIN_8
#define M2_FLAG_GPIO_Port GPIOC
#define M2_SW_Pin GPIO_PIN_9
#define M2_SW_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
#define USB_TX_Pin GPIO_PIN_9
#define USB_TX_GPIO_Port GPIOA
#define USB_RX_Pin GPIO_PIN_10
#define USB_RX_GPIO_Port GPIOA
#define PUMP7_PWM_Pin GPIO_PIN_12
#define PUMP7_PWM_GPIO_Port GPIOA
#define RS0_Pin GPIO_PIN_15
#define RS0_GPIO_Port GPIOA
#define RS4_Pin GPIO_PIN_10
#define RS4_GPIO_Port GPIOC
#define RS5_Pin GPIO_PIN_11
#define RS5_GPIO_Port GPIOC
#define M2_STCK_Pin GPIO_PIN_12
#define M2_STCK_GPIO_Port GPIOC
#define LED_MCU_CHECK_Pin GPIO_PIN_0
#define LED_MCU_CHECK_GPIO_Port GPIOD
#define M0_STCK_Pin GPIO_PIN_1
#define M0_STCK_GPIO_Port GPIOD
#define PUMP9_IN1_CW_Pin GPIO_PIN_2
#define PUMP9_IN1_CW_GPIO_Port GPIOD
#define PUMP9_IN2_CCW_Pin GPIO_PIN_3
#define PUMP9_IN2_CCW_GPIO_Port GPIOD
#define PUMP9_PWM_Pin GPIO_PIN_4
#define PUMP9_PWM_GPIO_Port GPIOD
#define LCD_TX_Pin GPIO_PIN_5
#define LCD_TX_GPIO_Port GPIOD
#define LCD_RX_Pin GPIO_PIN_6
#define LCD_RX_GPIO_Port GPIOD
#define PUMP10_IN1_CW_GPIO_Pin GPIO_PIN_7
#define PUMP10_IN1_CW_GPIO_Port GPIOD
#define RS1_Pin GPIO_PIN_3
#define RS1_GPIO_Port GPIOB
#define RS2_Pin GPIO_PIN_4
#define RS2_GPIO_Port GPIOB
#define RS3_Pin GPIO_PIN_5
#define RS3_GPIO_Port GPIOB
#define I2C_SCL_EEPROM_Pin GPIO_PIN_6
#define I2C_SCL_EEPROM_GPIO_Port GPIOB
#define I2C_SDA_EEPROM_Pin GPIO_PIN_7
#define I2C_SDA_EEPROM_GPIO_Port GPIOB
#define PUMP7_IN1_CW_Pin GPIO_PIN_8
#define PUMP7_IN1_CW_GPIO_Port GPIOB
#define PUMP7_IN2_CCW_Pin GPIO_PIN_9
#define PUMP7_IN2_CCW_GPIO_Port GPIOB
#define PUMP6_PWM_Pin GPIO_PIN_0
#define PUMP6_PWM_GPIO_Port GPIOE
#define PUMP6_IN1_CW_Pin GPIO_PIN_1
#define PUMP6_IN1_CW_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
