/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  Mateus Xavier
  * @version V1.0.0
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

/*************************************************************************************************************/
/* --- INCLUDES ---------------------------------------------------------------------------------------------*/

#include "stm8s.h"
#include "stm8s_it.h"
#include "stm8s_delay.h"
#include <string.h>

/*************************************************************************************************************/
/* --- DEFINE/MACRO -----------------------------------------------------------------------------------------*/

#define LED_A_PORT            GPIOA
#define LED_A_PIN             GPIO_PIN_3

/* ----------------------------------------------------------------------------------------------------------*/

#define SCL_PORT              GPIOB
#define SCL_PIN               GPIO_PIN_4

#define SDA_PORT              GPIOB
#define SDA_PIN               GPIO_PIN_5

/* ----------------------------------------------------------------------------------------------------------*/

#define RE_DT_PORT            GPIOA
#define RE_DT_PIN             GPIO_PIN_1

#define RE_CLK_PORT           GPIOA
#define RE_CLK_PIN            GPIO_PIN_2

/* ----------------------------------------------------------------------------------------------------------*/

#define MCP4725_MAX_VALUE     4095

/* ----------------------------------------------------------------------------------------------------------*/

#define TIM4_PERIOD           255

/* ----------------------------------------------------------------------------------------------------------*/

#define MCP4725_ADDRESS       0xC2

/* ----------------------------------------------------------------------------------------------------------*/

#define CCR3_Val  ((uint16_t)1023)

/* ----------------------------------------------------------------------------------------------------------*/

#define LCD_RS                GPIOD, GPIO_PIN_1
#define LCD_EN                GPIOD, GPIO_PIN_3
#define LCD_DB4               GPIOC, GPIO_PIN_4
#define LCD_DB5               GPIOC, GPIO_PIN_5
#define LCD_DB6               GPIOC, GPIO_PIN_6
#define LCD_DB7               GPIOC, GPIO_PIN_7


#include "stm8s_LCD_16x2.h"

/*************************************************************************************************************/
/* --- MACRO ------------------------------------------------------------------------------------------------*/

#define READ_RE_CLK           GPIO_ReadInputPin(RE_CLK_PORT, RE_CLK_PIN)
#define READ_RE_DT            GPIO_ReadInputPin(RE_DT_PORT, RE_DT_PIN)

/* ----------------------------------------------------------------------------------------------------------*/

#define LED_A_ON              GPIO_WriteHigh(LED_A_PORT, LED_A_PIN)
#define LED_A_OFF             GPIO_WriteLow(LED_A_PORT, LED_A_PIN)
#define LED_A_TOGGLE          GPIO_WriteReverse(LED_A_PORT, LED_A_PIN)

/*************************************************************************************************************/
/* --- VARIABLES --------------------------------------------------------------------------------------------*/

uint16_t MCP4725_value = 0;
uint8_t MCP4725_UpdateFlag = 10;
char buffer[20];
float MCP4725_Voltage = 0;
char MCP4725_String_Voltage[5];
uint16_t Input_Current_Bit_Value = 0;
float Input_Current_Value = 0;
char Input_Current_String_Value[5];
uint16_t Input_Voltage_Bit_Value = 0;
float Input_Voltage_Value = 0;
char Input_Voltage_String_Value[5];
int old_num = 0;
int current_num = 0;
char Temp_buffer[6];

/*************************************************************************************************************/
/* --- FUNCTION ---------------------------------------------------------------------------------------------*/

void MCU_init(void);
void OSC_init(void);
void GPIO_init(void);
void I2Cinit(void);
void TIM1init(void);
void TMR4_init(void);

/* ----------------------------------------------------------------------------------------------------------*/

void MCP4725_write(uint16_t data);
void MCP4725_valueUpdate(void);

uint16_t Read_ADC1_Channel_3(void);
uint16_t Read_ADC1_Channel_4(void);

/* ----------------------------------------------------------------------------------------------------------*/

void BlinkLED(void);

/* ----------------------------------------------------------------------------------------------------------*/

void RotaryEncoderHandler(void);

/* ----------------------------------------------------------------------------------------------------------*/

void Delay_100us(void);
void Delay_ms(unsigned int VezesT);

void Float_to_String(float Value, char *Converted_Value);


/*************************************************************************************************************/
/* --- INTERRUPT HANDLER ------------------------------------------------------------------------------------*/

/* External Interrupt function PORTC ------------------------------------------------------------------------*/
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3)
{
  RotaryEncoderHandler();
}

/* Interrupt function TIMER 4 -------------------------------------------------------------------------------*/
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
 {  
    
    //Input_Voltage_Bit_Value = Read_ADC1_Channel_3();

    //Input_Current_Bit_Value = Read_ADC1_Channel_4();                // Captura o valor do ADC

    TIM4_ClearFlag(TIM4_FLAG_UPDATE);                               // Limpa o flag do timer4
 }

/*************************************************************************************************************/
/* --- MAIN LOOP --------------------------------------------------------------------------------------------*/

void main(void)
{

  MCU_init();                  // Initializing configurations
  MCP4725_valueUpdate();

  Lcd_Clear();
  Lcd_Set_Cursor(1,1);
  Lcd_Print_String("Tensao: ");
  Lcd_Set_Cursor(2,1);
  Lcd_Print_String("Corrente: ");
  

  /* Infinite loop */
  while (1)
  {
    /*
    //MCP4725_write(0x82);

    I2C_GenerateSTART(ENABLE);
    
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(0x96, I2C_DIRECTION_TX);

    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    I2C_SendData(0x01);

    //while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING));

    I2C_SendData(0xC4);

    //while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_SendData(0x83);

    //while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTOP(ENABLE);





    I2C_GenerateSTART(ENABLE);
    
    //while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(0x90, I2C_DIRECTION_RX);

    //while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    I2C_SendData(0x01);

    //while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING));

    I2C_SendData(0x84);

    //while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_SendData(0x83);

    //while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTOP(ENABLE);



    I2C_GenerateSTART(ENABLE);
    
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(0x90, I2C_DIRECTION_RX);

    //while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    
    current_num = I2C_ReceiveData();

    //while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    current_num = (current_num << 4) & I2C_ReceiveData();

    //while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    I2C_GenerateSTOP(ENABLE);
*/

    //Input_Voltage_Value = ((377.0/1024.0)*Input_Voltage_Bit_Value);
    //Float_to_String(Input_Voltage_Value, Input_Voltage_String_Value);

    //Input_Current_Value = ((490.0/1024.0)*Input_Current_Bit_Value);
    //Float_to_String(Input_Current_Value, Input_Current_String_Value);

    //MCP4725_Voltage = ((490.0/4096.0)*MCP4725_value) + 3;
    //Float_to_String(MCP4725_Voltage, MCP4725_String_Voltage);

    //Lcd_Set_Cursor(1,11);
    //Lcd_Print_String(Input_Voltage_String_Value);

    //current_num = (int)((490.0/1023.0)*Read_ADC1_Channel_3());

    current_num = Read_ADC1_Channel_3();
    current_num = (int)((0.7 * current_num) + (0.3 * old_num));

    char dec0 = (current_num % 10) + 0x30;
    current_num = current_num / 10;
    char dec1 = (current_num % 10) + 0x30;
    current_num = current_num / 10;
    char dec2 = (current_num % 10) + 0x30;
    current_num = current_num / 10;
    char dec3 = (current_num % 10) + 0x30;
    current_num = current_num / 10;
    char dec4 = (current_num % 10) + 0x30;
    current_num = current_num / 10;

    Temp_buffer[0] = dec4;
    Temp_buffer[1] = dec3;
    Temp_buffer[2] = dec2;
    Temp_buffer[3] = dec1;
    Temp_buffer[4] = dec0;
    Temp_buffer[5] = '\0';

    Lcd_Set_Cursor(2,11);
    Lcd_Print_String(Temp_buffer);

    old_num = current_num;

    Delay_ms(10);


  } 
}

/*************************************************************************************************************/
/* --- FUNCTION ---------------------------------------------------------------------------------------------*/

/**
  * @brief  Microcontroller initialization function
  */
void MCU_init(void){
  OSC_init();                                      // Initializing clock configutation
  GPIO_init();                                     // Initializing GPIO configuration
  I2Cinit();                                      // Initializing I2C configuration
  TIM1init();
  TMR4_init();                                     // Initializing TMR4 configuration
  Lcd_Begin();

  Delay_ms(500);                                  // Wait for registers to stable
  enableInterrupts();
}

/**
  * @brief  High Speed Internal Oscillator / 2 Prescaler -> 8 MHz Clock
  */
void OSC_init(void){
	CLK_DeInit();								                    // Reseta as config. de clock
	CLK_HSECmd(DISABLE);						                    // Desabilita HSE
  CLK_HSICmd(ENABLE);							                    // Habilita HSI - 16Mhz
	while(CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == 0);
	CLK_ClockSwitchCmd(ENABLE);
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV4);      // 16Mhz / 4 -> Fmaster = 4MHz
	CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV2);            // 4MHz  -> Fcup = 4MHz
	CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE,CLK_CURRENTCLOCKSTATE_ENABLE);
	
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE); 

  // CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE); 
	// CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, DISABLE); 
	// CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, DISABLE); 
	// CLK_PeripheralClockConfig(CLK_PERIPHERAL_AWU, DISABLE); 
	// CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, ENABLE); 
	// CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, DISABLE);   
	

}

/**
  * @brief  General Ports configuration
  */
void GPIO_init(void){

  GPIO_DeInit(GPIOC);
  GPIO_DeInit(GPIOB);

  GPIO_Init(LED_A_PORT, LED_A_PIN, GPIO_MODE_OUT_PP_LOW_FAST);

  GPIO_Init(RE_DT_PORT, RE_DT_PIN, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(RE_CLK_PORT, RE_CLK_PIN, GPIO_MODE_IN_PU_IT);

  GPIO_Init(SCL_PORT, SCL_PIN, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(SDA_PORT, SDA_PIN, GPIO_MODE_IN_PU_NO_IT);

}

/**
  * @brief  Interrupts configurations
  */
void INT_init(void){
  
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA, EXTI_SENSITIVITY_FALL_ONLY);
  ITC_SetSoftwarePriority(ITC_IRQ_PORTA, ITC_PRIORITYLEVEL_1);

}

/**
  * @brief  Timer 4 used as basic 8 bit timer, frequency 31.372 kHz
  */
void TMR4_init(void){

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

/**
  * @brief  I2C Speed: 400 kHz, I2C_DUTYCYCLE_2, I2C_ADDMODE_7BIT
  */
void I2Cinit(void){

  I2C_DeInit();

  CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);

  uint8_t Input_Clock = 0;
  Input_Clock = CLK_GetClockFreq() / 1000000;

  I2C_Init(400000, 0xA0, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, Input_Clock);

  I2C_Cmd(ENABLE);
  

}

/**
  * @brief  Read_ADC1_Channel_3
  */
uint16_t Read_ADC1_Channel_3(void){
  ADC1_DeInit();                                      // Reseta qualquer configuracao anterior

  uint16_t temp;

  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,           // Conversao modo continuo
            ADC1_CHANNEL_3,                           // Seleciona canal 4
            ADC1_PRESSEL_FCPU_D2,                     // Prescaller de 1:4
            ADC1_EXTTRIG_GPIO,                        // Tigger externo desabilitado
            DISABLE,
            ADC1_ALIGN_RIGHT,                         // Configurado em 10 bits
            ADC1_SCHMITTTRIG_CHANNEL3,                // Canal schmitt trigger desabilitado
            DISABLE);
  ADC1_Cmd(ENABLE); 
  ADC1_StartConversion();
  while(!ADC1_GetFlagStatus(ADC1_FLAG_EOC));                      // Espera finalizar a conversao AD
  temp = (int16_t)ADC1_GetConversionValue();
  ADC1_ClearFlag(ADC1_FLAG_EOC); 
  ADC1_Cmd(DISABLE);
  return temp;

}

/**
  * @brief  Read_ADC1_Channel_4
  */
uint16_t Read_ADC1_Channel_4(void){
  ADC1_DeInit();                                      // Reseta qualquer configuracao anterior

  uint16_t temp;

  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,           // Conversao modo continuo
            ADC1_CHANNEL_4,                           // Seleciona canal 4
            ADC1_PRESSEL_FCPU_D4,                     // Prescaller de 1:4
            ADC1_EXTTRIG_GPIO,                        // Tigger externo desabilitado
            DISABLE,
            ADC1_ALIGN_RIGHT,                         // Configurado em 10 bits
            ADC1_SCHMITTTRIG_CHANNEL4,                // Canal schmitt trigger desabilitado
            DISABLE);
                          // M?dulo ADC desabilitado
  ADC1_Cmd(ENABLE); 
  ADC1_StartConversion();
  while(!ADC1_GetFlagStatus(ADC1_FLAG_EOC));                      // Espera finalizar a conversao AD
  temp = (int16_t)ADC1_GetConversionValue();
  ADC1_ClearFlag(ADC1_FLAG_EOC); 
  ADC1_Cmd(DISABLE);
  return temp;

}

/**
  * @brief  Change voltage value of DAC
  * @param  Voltage Value of DAC
  */
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

/**
  * @brief  MCP4725_valueUpdate
  */
void MCP4725_valueUpdate(void){
  MCP4725_write(MCP4725_value);
}

/**
  * @brief  RotaryEncoderHandler
  */
void RotaryEncoderHandler(void)
{
    Delay_ms(1);
    if(!READ_RE_CLK) {
	   if((!READ_RE_DT)){
        BlinkLED();
        if(MCP4725_value - 1 >= MCP4725_MAX_VALUE) MCP4725_value = 4095;
        else MCP4725_value = MCP4725_value - 1;
     } 
		else{
      if(MCP4725_value + 1 >= MCP4725_MAX_VALUE) MCP4725_value = 0;
      else MCP4725_value = MCP4725_value + 1;
    } 
	  }
    MCP4725_valueUpdate();
}

/**
  * @brief  RotaryEncoderHandler
  */
void Float_to_String(float Value, char *Converted_Value){
  int current_num = (int)Value;

  char dec1 = (current_num % 10) + 0x30;
  current_num = current_num / 10;
  char dec2 = (current_num % 10) + 0x30;
  current_num = current_num / 10;
  char dec3 = (current_num % 10) + 0x30;
  
  Converted_Value[0] = dec3;
  Converted_Value[1] = '.';
  Converted_Value[2] = dec2;
  Converted_Value[3] = dec1;
  Converted_Value[4] = '\0';

}

/**
  * @brief  Blink LED with delays of 250 ms
  */
void BlinkLED(){
    LED_A_ON;
    Delay_ms(250);                 // Wait for 250ms 
    LED_A_OFF;
    Delay_ms(250);                 // Wait for 250ms
}


#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line current_number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source current_number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line current_number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
