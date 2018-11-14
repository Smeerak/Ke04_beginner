/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    empty.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKE04Z4.h"


//Counter values for PWM motor control
#define MOTOR_OFF 0
#define MOTOR_LEFT  1
#define MOTOR_RIGHT 2


volatile uint32_t g_systickCounter;

void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while(g_systickCounter != 0U)
    {
    }
}

void flash_up()
{
	GPIO_PinWrite(kGPIO_PORTB, 1U, 1);
	GPIO_PinWrite(kGPIO_PORTB, 2U, 0);

	SysTick_DelayTicks(100U);

	GPIO_PinWrite(kGPIO_PORTB, 1U, 0);
	GPIO_PinWrite(kGPIO_PORTB, 2U, 0);

}


void flash_down()
{
	GPIO_PinWrite(kGPIO_PORTB, 1U, 0);
	GPIO_PinWrite(kGPIO_PORTB, 2U, 1);

	SysTick_DelayTicks(100U);

	GPIO_PinWrite(kGPIO_PORTB, 1U, 0);
	GPIO_PinWrite(kGPIO_PORTB, 2U, 0);

}

void pwt_init(PWT_Type pwt_base, SIM_Type *sim_base)
{

//	sim_base->PINSEL |= SIM_PINSEL_PWTCLKPS(0);  //assign TCLK1 to PWT
//	sim_base->SCGC |= SIM_SCGC_PWT(1); //enable clk to pwt

	//there is no pin select as PWT is only available on a single pin

}

void pwt_control()
{

	//activate
	//sim_base->    SIM_SCGC_PWT(x)
}

void ftm_init(FTM_Type *ftm_base, SIM_Type *sim_base, uint32_t val)
{


	sim_base->PINSEL |= SIM_PINSEL_FTM0PS1(1); // pin select PTB3
	sim_base->SCGC |= SIM_SCGC_FTM0_MASK; // enabled clock for FTM0 (system clock gating)

	//setup for FTM0 Channel 1
	ftm_base->SC |= FTM_SC_PS(6); // prescale 32
	ftm_base->CNT = 0;
	ftm_base->MOD = 7500; //some value, we will work out frequencies later on (looking for 50hz)  7000 == 53.2Hz
	ftm_base->CONTROLS[1].CnSC |= FTM_CnSC_MSB_MASK;   //edge aligned
	ftm_base->CONTROLS[1].CnSC |=  FTM_CnSC_ELSB_MASK; // high-true pulses

	ftm_base->CONTROLS[1].CnV &= FTM_CnV_VAL(0);
	ftm_base->CONTROLS[1].CnV |= FTM_CnV_VAL(val); // ratio of MOD == duty cycle  2.0ms / 90.2% @ 730

	//ftm_base->CNTIN &= FTM_CNTIN_INIT(0);

	//ftm_base->SC |= FTM_SC_CLKS(1); // use system clock
}


void ftm_control(int control, FTM_Type *ftm_base)
{

	//7500 MOD
	//2ms 730 VAL
	//1ms 365 VAL

	switch(control)
	{
	case 0:
		ftm_base->SC &= FTM_SC_CLKS(0); // stop clock
		break;
	case 1:
		ftm_base->SC &= FTM_SC_CLKS(0); // stop FTM clock
		ftm_base->SC |= FTM_SC_PS(6); // prescale 32 -- this is required, investigate
		ftm_base->CNT = 0;
		ftm_base->MOD = 7500;//
		ftm_base->CONTROLS[1].CnV &= FTM_CnV_VAL(0); // clear register
		ftm_base->CONTROLS[1].CnV |= FTM_CnV_VAL(365);
		ftm_base->SC |= FTM_SC_CLKS(1); // use system clock, start FTM
		break;
	case 2:
		ftm_base->SC &= FTM_SC_CLKS(0); // stop FTM clock
		ftm_base->SC |= FTM_SC_PS(6); // prescale 32 -- this is required, investigate
		ftm_base->CNT = 0;
		ftm_base->MOD = 7500;//
		ftm_base->CONTROLS[1].CnV &= FTM_CnV_VAL(0); // clear register
		ftm_base->CONTROLS[1].CnV |= FTM_CnV_VAL(730);
		ftm_base->SC |= FTM_SC_CLKS(1); // use system clock, start FTM
		break;
	}

}


int main(void) {

	int j = 0;

	/* Init board hardware. */
    BOARD_InitBootPins();  // configure GPIO
    ftm_init(FTM0, SIM, 365);   // configure and start FTM

   /* Set systick reload value to generate 1ms interrupt */
   if(SysTick_Config(SystemCoreClock / 1000U))
   {
	   while(1)
	   {
	   }
   }

   	   for(j = 0 ; j < 5 ; j ++)
   	   {
   		   SysTick_DelayTicks(100U);

   		  flash_up();

   		SysTick_DelayTicks(100U);

   		  flash_down();
   	   }

   	   GPIO_PinWrite(kGPIO_PORTB, 1U, 0);
   	   GPIO_PinWrite(kGPIO_PORTB, 2U, 0);

       uint8_t key_up_handler = 1;
	   uint8_t key_dn_handler = 1;

	   uint8_t menu_pos = 0;

	   uint8_t action_state = 0; //determine if an action needs to occur

	   while (1)
	   {
		   SysTick_DelayTicks(10U);

		   //determine a key up press
		   if(GPIO_PinRead(kGPIO_PORTB, 6) == 0)
		   {
			if(key_up_handler)
			{
				flash_up(); //flash the led as soon as the button is pushed and the system is waiting for a release

				//if we hit the same button, stop the FTM
				if(menu_pos == 1)
					menu_pos = 0;
				else
					menu_pos = 1;

				key_up_handler = 0;
				action_state = 1;

			}
		   }
		   else //release button up or normal up state
		   {
			//do nothing
			key_up_handler = 1;

		   }
		   //determine a key down press
		   if(GPIO_PinRead(kGPIO_PORTA, 1) == 0)
		   {
			if(key_dn_handler)
			{
				flash_down();

				//if we hit the same button, stop the FTM
				if(menu_pos == 2)
					menu_pos = 0;
				else
					menu_pos = 2;

				key_dn_handler = 0;
				action_state = 1;
			}
		   }
		   else
		   {
			//do nothing
			   key_dn_handler = 1;

		   }

		   //handle the menu actions
		   if(action_state == 1) // execute the command once, and then wait for a new command
		   {
			   switch(menu_pos)
			   {
			   case 0:
				   ftm_control(0, FTM0);
				   GPIO_PinWrite(kGPIO_PORTB, 1U, 0);
				   GPIO_PinWrite(kGPIO_PORTB, 2U, 1);
				   action_state = 0;
				   break;
			   case 1:
				   ftm_control(2, FTM0);
				   GPIO_PinWrite(kGPIO_PORTB, 1U, 0);
				   GPIO_PinWrite(kGPIO_PORTB, 2U, 0);
				   action_state = 0;
				   break;

			   case 2:
				   ftm_control(1, FTM0);
			       GPIO_PinWrite(kGPIO_PORTB, 1U, 0);
				   GPIO_PinWrite(kGPIO_PORTB, 2U, 0);
				   action_state = 0;
			       break;
			   }
			   //actions are resolved, ignore until next action
		   }
	   }
   }
