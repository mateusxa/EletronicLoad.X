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

#define MCP4725_ADDRESS       0xC2

/* Functions -----------------------------------------------------------------*/
void MCUinit(void);
void CLKinit(void);
void GPIOinit(void);
void I2Cinit(void);
// -------------------------------------------------------------------------- //
void MCP4725_write(uint16_t data);
void BlinkLED(void);

void Delay_100us(void);
void Delay_ms(unsigned int VezesT);


/* Main function -------------------------------------------------------------*/
void main(void)
{
  MCUinit();                  // Initializing configurations

  /* Infinite loop */
  while (1)
  {
    MCP4725_write(2045);
    Delay_ms(500);

  }
}

/* MCU function -------------------------------------------------------------*/
void MCUinit(void){
  CLKinit();                                      // Initializing clock configutation
  GPIOinit();                                     // Initializing GPIO configuration
  Delay_ms(500);                                  // Wait for registers to stable
  I2Cinit();                                      // Initializing I2C configuration
}

/* CLK function -------------------------------------------------------------*/
void CLKinit(void){
	CLK_DeInit();								                    // Reseta as config. de clock
	CLK_HSECmd(DISABLE);						                    // Desabilita HSE
        CLK_HSICmd(ENABLE);							                    // Habilita HSI - 16Mhz
	while(CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == 0);
	CLK_ClockSwitchCmd(ENABLE);
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV4);      // 16Mhz / 4 -> Fmaster = 4MHz
	CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);            // 4MHz / 4 -> Fcup = 1MHz
	CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE,CLK_CURRENTCLOCKSTATE_ENABLE);
	
        CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, DISABLE); 
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, DISABLE); 
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_AWU, DISABLE); 
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, DISABLE); 
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, DISABLE); 
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2, DISABLE);   
	CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, DISABLE);

}

/* GPIO function -------------------------------------------------------------*/
void GPIOinit(void){
  GPIO_DeInit(GPIOC);                             // Reset GPIO config
  GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
  
  GPIO_DeInit(GPIOB);                             // Reset GPIO config
  GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_IN_PU_NO_IT);
}

/* I2C function -------------------------------------------------------------*/
void I2Cinit(void){

  I2C_DeInit();

  uint8_t Input_Clock = 0;
  Input_Clock = CLK_GetClockFreq() / 1000000;

  I2C_Init(100000, 0xA0, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, Input_Clock);

  I2C_Cmd(ENABLE);
  

}

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

/* BlinkLED function -------------------------------------------------------------*/
void BlinkLED(){
    GPIO_WriteHigh(GPIOC, GPIO_PIN_3);
    Delay_ms(250);                 // Wait for 250ms 
    GPIO_WriteLow(GPIOC, GPIO_PIN_3);
    Delay_ms(250);                 // Wait for 250ms
}


/* Delay 100us function -------------------------------------------------------------*/
// Tempo = 4cy + 2cy + 22*4cy + 5cy = 99cy => 99us
void Delay_100us(void)
{
    __asm(  "push A\n"						
            "ld A,#22\n"					
            "start:	nop\n"							
            "dec A\n"						
            "jrne start\n"					
            "pop A\n"						
            "ret\n"
         );
}

/* Delay_ms function -------------------------------------------------------------*/
void Delay_ms(unsigned int VezesT)
{
	unsigned int i;
	
	for(i = 0; i < VezesT; i++)
	{
		Delay_100us();
		Delay_100us();
		Delay_100us();
		Delay_100us();
		Delay_100us();
		Delay_100us();
		Delay_100us();
		Delay_100us();
		Delay_100us();
		Delay_100us();
	}
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
