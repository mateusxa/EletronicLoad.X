/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.3.0
  * @date    16-June-2017
  * @brief   Main program body
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

void MCUinit(void);
void CLKinit(void);
void GPIOinit(void);
void Delay_100us(void);
void Delay_ms(unsigned int VezesT);
void BlinkLED(void);

void main(void)
{

  MCUinit();
  /* Infinite loop */
  while (1)
  {

    BlinkLED();
    

  }
  
}


void MCUinit(void){
  CLKinit();                                      // Initializing clock configutation
  GPIOinit();                                     // Initializing GPIO configuration
  Delay_ms(500);                                  // Wait for registers to stable
}

void CLKinit(void){
	CLK_DeInit();								                    // Reset Inital config
	
  CLK->ICKR |= 0x01;                              // Enabling HSI -> 16MHz
  while((CLK->ICKR & 0x02) != 0x02);              // Wait for HSI to be ready
  CLK->SWCR |= 0x02;                              // Enable Switching Clock
  CLK->SWR = 0xE1;                                // Select HSI as Master Clock
  CLK->CKDIVR |= 0x12;                            // Master - HSI/4(10) -> 4 MHz | CPU - MAster/4 -> 1 MHz
}

void GPIOinit(void){
  GPIO_DeInit(GPIOC);                             // Reset GPIO config
  
  GPIOC->DDR = 0x08;                              // Set PC3 as Output
  GPIOC->CR1 = 0xFF;                              // All Pull-up
  GPIOC->CR2 = 0;                                 // Reseting register
}



void BlinkLED(){
    GPIOC->ODR &= ~(0x08);          // Set LED Low
    Delay_ms(250);                 // Wait for 250ms 
    GPIOC->ODR |= 0x08;             // Set LED High
    Delay_ms(250);                 // Wait for 250ms
    GPIOC->ODR &= ~(0x08);          // Set LED Low Again
}


//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------

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
