#include "stm8s_delay.h"

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