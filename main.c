/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  Mateus Xavier
  * @version V2.3.0
  * @date    03-10-2021
  * @brief   Eletronic Load
   ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8s_it.h"
#include "stm8s_delay.h"
#include <string.h>


/* Defines -------------------------------------------------------------------*/
#define LED_A_PORT            GPIOA
#define LED_A_PIN             GPIO_PIN_3

//#define LED_B_PORT            GPIOA
//#define LED_B_PIN             GPIO_PIN_2
/* ---------------------------------------------------------------------------*/
#define SCL_PORT              GPIOB
#define SCL_PIN               GPIO_PIN_4

#define SDA_PORT              GPIOB
#define SDA_PIN               GPIO_PIN_5
/* ---------------------------------------------------------------------------*/
#define RE_DT_PORT            GPIOA
#define RE_DT_PIN             GPIO_PIN_1

#define RE_CLK_PORT           GPIOA
#define RE_CLK_PIN            GPIO_PIN_2
/* ---------------------------------------------------------------------------*/
#define MCP4725_MAX_VALUE     4095
/* ---------------------------------------------------------------------------*/
#define TIM4_PERIOD           255
/* ---------------------------------------------------------------------------*/
#define MCP4725_ADDRESS       0xC2
/* ---------------------------------------------------------------------------*/
#define LCD_RS                GPIOD, GPIO_PIN_1
#define LCD_EN                GPIOD, GPIO_PIN_2
#define LCD_DB4               GPIOC, GPIO_PIN_4
#define LCD_DB5               GPIOC, GPIO_PIN_5
#define LCD_DB6               GPIOC, GPIO_PIN_6
#define LCD_DB7               GPIOC, GPIO_PIN_7

#include "stm8s_LCD_16x2.h"

//Variable declarations


/* Macros -------------------------------------------------------------------*/
#define READ_RE_CLK           GPIO_ReadInputPin(RE_CLK_PORT, RE_CLK_PIN)
#define READ_RE_DT            GPIO_ReadInputPin(RE_DT_PORT, RE_DT_PIN)
/* ---------------------------------------------------------------------------*/
#define LED_A_ON              GPIO_WriteHigh(LED_A_PORT, LED_A_PIN)
#define LED_A_OFF             GPIO_WriteLow(LED_A_PORT, LED_A_PIN)
#define LED_A_TOGGLE          GPIO_WriteReverse(LED_A_PORT, LED_A_PIN)

//#define LED_B_ON              GPIO_WriteHigh(LED_B_PORT, LED_B_PIN)
//#define LED_B_OFF             GPIO_WriteLow(LED_B_PORT, LED_B_PIN)
//#define LED_B_TOGGLE          GPIO_WriteReverse(LED_B_PORT, LED_B_PIN)

/* Variables -------------------------------------------------------------------*/
uint16_t MCP4725_value = 0;
uint8_t MCP4725_UpdateFlag = 10;
char buffer[20];
uint16_t MCP4725_Voltage_value = 0;

/* ---------------------------------------------------------------------------*/



/* Functions -----------------------------------------------------------------*/
void MCUinit(void);
void CLKinit(void);
void GPIOinit(void);
void I2Cinit(void);
void TMR4init(void);
/* ---------------------------------------------------------------------------*/
void MCP4725_write(uint16_t data);
void MCP4725_valueUpdate(void);
/* ---------------------------------------------------------------------------*/
void BlinkLED_B(void);
void BlinkLED_A(void);
/* ---------------------------------------------------------------------------*/
void RotaryEncoderHandler(void);
/* ---------------------------------------------------------------------------*/
void Delay_100us(void);
void Delay_ms(unsigned int VezesT);

char* itoa(int value, char* result, int base);


/* INTERRUPTS -------------------------------------------------------------*/

/* External Interrupt function PORTC --------------------------------------*/
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3)
{
  RotaryEncoderHandler();
}

/* Interrupt function TIMER 4 --------------------------------------*/
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
 {  
    //if(!MCP4725_UpdateFlag) MCP4725_UpdateFlag = 1;


    TIM4_ClearFlag(TIM4_FLAG_UPDATE);                               // Limpa o flag do timer4
 }

/* Main function -------------------------------------------------------------*/
void main(void)
{

  MCUinit();                  // Initializing configurations
  MCP4725_valueUpdate();

  Lcd_Clear();
  Lcd_Set_Cursor(1,1);
  Lcd_Print_String("Tensao: ");
  Lcd_Set_Cursor(2,1);
  Lcd_Print_String("Corrente: ");
  

  /* Infinite loop */
  while (1)
  {
    MCP4725_Voltage_value = ((5*MCP4725_value)/4096)*100;
    itoa(MCP4725_Voltage_value, buffer, 10);
    Lcd_Set_Cursor(2,11);
    Lcd_Print_String(buffer);
    Delay_ms(10);


    /*
    MCP4725_write(1000);
    
    
    BlinkLED_A();
    Delay_ms(500);
    BlinkLED_B();
    Delay_ms(500);
    */
  } 
}


/* MCU function -------------------------------------------------------------*/
void MCUinit(void){
  CLKinit();                                      // Initializing clock configutation
  GPIOinit();                                     // Initializing GPIO configuration
  I2Cinit();                                      // Initializing I2C configuration
  TMR4init();                                     // Initializing TMR4 configuration
  Lcd_Begin();

  Delay_ms(500);                                  // Wait for registers to stable
  enableInterrupts();
}

/* CLK function -------------------------------------------------------------*/
void CLKinit(void){
	CLK_DeInit();								                    // Reseta as config. de clock
	CLK_HSECmd(DISABLE);						                    // Desabilita HSE
  CLK_HSICmd(ENABLE);							                    // Habilita HSI - 16Mhz
	while(CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == 0);
	CLK_ClockSwitchCmd(ENABLE);
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV4);      // 16Mhz / 4 -> Fmaster = 4MHz
	CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);            // 4MHz  -> Fcup = 4MHz
	CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE,CLK_CURRENTCLOCKSTATE_ENABLE);
	
  
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, DISABLE); 
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, DISABLE); 
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_AWU, DISABLE); 
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, DISABLE); 
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, DISABLE); 
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, DISABLE);   
	

}

/* GPIO function -------------------------------------------------------------*/
void GPIOinit(void){

  GPIO_DeInit(GPIOC);
  GPIO_DeInit(GPIOB);

  GPIO_Init(LED_A_PORT, LED_A_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  //GPIO_Init(LED_B_PORT, LED_B_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  
  GPIO_Init(RE_DT_PORT, RE_DT_PIN, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(RE_CLK_PORT, RE_CLK_PIN, GPIO_MODE_IN_PU_IT);

  GPIO_Init(SCL_PORT, SCL_PIN, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(SDA_PORT, SDA_PIN, GPIO_MODE_IN_PU_NO_IT);
/*
  GPIO_Init(LCD_RS_PORT, LCD_RS_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(LCD_ENABLE_PORT, LCD_ENABLE_PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(LCD_DATA_PORT, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(LCD_DATA_PORT, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(LCD_DATA_PORT, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(LCD_DATA_PORT, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_SLOW);
*/

}

/* INT function -------------------------------------------------------------*/
void INTinit(void){
  
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_FALL_ONLY);
  ITC_SetSoftwarePriority(ITC_IRQ_PORTA, ITC_PRIORITYLEVEL_1);

}

/* TMR4init function -------------------------------------------------------------*/
void TMR4init(void){

    TIM4_DeInit();

    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
    /* Time base configuration */
    TIM4_TimeBaseInit(TIM4_PRESCALER_1, TIM4_PERIOD);
    /* Clear TIM4 update flag */
    TIM4_ClearFlag(TIM4_FLAG_UPDATE);
    /* Enable update interrupt */
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);

    /* Enable TIM4 */
    TIM4_Cmd(ENABLE);

    
}

/* I2C function -------------------------------------------------------------*/
void I2Cinit(void){

  I2C_DeInit();

  CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);

  uint8_t Input_Clock = 0;
  Input_Clock = CLK_GetClockFreq() / 1000000;

  I2C_Init(400000, 0xA0, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, Input_Clock);

  I2C_Cmd(ENABLE);
  

}

/* MCP4725_write function -------------------------------------------------------------*/
void MCP4725_write(uint16_t data){

    uint8_t FIRST_NIMBLE = (data >> 8);

    uint8_t SECOND_THIRD_NIMBLE = (data & 0xFF);

    I2C_GenerateSTART(ENABLE);
    
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(MCP4725_ADDRESS, I2C_DIRECTION_TX);

    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    I2C_SendData(FIRST_NIMBLE);

    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING));

    I2C_SendData(SECOND_THIRD_NIMBLE);

    while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTOP(ENABLE);
}

/* MCP4725_valueUpdate function -------------------------------------------------------------*/
void MCP4725_valueUpdate(void){
  MCP4725_write(MCP4725_value);
}

/* RotaryEncoderHandler function -------------------------------------------------------------*/
void RotaryEncoderHandler(void)
{
    Delay_ms(1);
    if(!READ_RE_CLK) {
	   if((!READ_RE_DT)){
        BlinkLED_A();
        if(MCP4725_value - 1 >= MCP4725_MAX_VALUE) MCP4725_value = 4095;
        else MCP4725_value = MCP4725_value - 1;
     } 
		else{
      BlinkLED_B();
      if(MCP4725_value + 1 >= MCP4725_MAX_VALUE) MCP4725_value = 0;
      else MCP4725_value = MCP4725_value + 1;
    } 
	  }
    MCP4725_valueUpdate();
}

/* BlinkLED_A function -------------------------------------------------------------*/
void BlinkLED_A(){
    LED_A_ON;
    Delay_ms(250);                 // Wait for 250ms 
    LED_A_OFF;
    Delay_ms(250);                 // Wait for 250ms
}

/* BlinkLED_B function -------------------------------------------------------------*/
void BlinkLED_B(){
    //LED_B_ON;
    Delay_ms(250);                 // Wait for 250ms 
    //LED_B_OFF;
    Delay_ms(250);                 // Wait for 250ms
}


/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
 */
char* itoa(int value, char* result, int base) {
    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
