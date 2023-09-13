/*
===============================================================================
 Name        : 34_Day71_30112021_TIMER005.c

 Description : Program for Timer's Counter mode to capture external clock pulses
 	 	 	   and calculate the frequency of the external clock pulses.

 Timer 0 is configured in timer mode for generating interrupts on every 100ms.
 Timer mode is nothing but counting the peripheral clock pulses.
 Timer 0 prescalar configured for 1ms

 Timer 2 is configured in timer mode for generating interrupt on every second.
 Hertz means number of pulses per second. So Timer 2 helps in measuring pulses per second.
 Timer 2 prescalar configured for 1ms

 Timer 1 is configured in counter mode for counting the external clock pulses.
 Timer 1 prescalar is configured as 0 for counting every falling edge of a pulse in TC.
 So counter mode is nothing but counting the external clock pulses.

 For this purpose CAP1.1 was configured in counter mode and we used the GPIO pin P0.7 to give the pulses to CAP1.1.
 So after every interrupt (1sec) of timer 2, timer 1 TC value is stored into a variable.
 and after that timer 1 will be reset and re enabled again.
 Counting external pulses (ie. frequency).

 Clock generation/ external pulse is coming from P0.7 and CAP1.1 (P1.19) captures this frequency.


 TODO        :

 Layered Architecture used for this project
 ************************************
 Application layer- 34_Day71_30112021_TIMER005.c
 *******************************************************************
 Board layer -  configboard.h, led.c/.h, buttonint.c/.h
 *******************************************************************
 Low level drivers or chip level - pinmux.c/.h,timer.c/.h, gpio.c/.h
 	 	 	 	 	 	 	 	   extint.c/.h
 *******************************************************************
 Hardware
 *******************************************************************
===============================================================================
*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
#include "pinmux.h"
#include "uart.h"
#include "timer.h"
#include "led.h"
#include "configboard.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* For testing the timer 2 handler is working perfectly */
#define TESTING_TIMER2HANDLER 1

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
volatile uint8_t  uartprintflag=0;
volatile uint32_t measuredfrequency=0;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Clock generator interrupt handler for TIMER0
  * @retval none
  */
void vClockGenerator(uint32_t interruptsource)
{
		static uint8_t ledstate=LED_OFF;
		switch (interruptsource) {
		case MR0INT:
			if (ledstate == LED_OFF)
			{
				vLedOn(LED_0);
				/* Need to connect this pin with P1.19 (CAP 1.1) */
				vGpioSetPin(PORT0,PIN7);
			}
			else if (ledstate == LED_ON)
			{
				vLedOff(LED_0);
				/* Need to connect this pin with P1.19 (CAP 1.1) */
				vGpioClrPin(PORT0,PIN7);
			}
			ledstate ^= LED_ON;
			break;
		case MR1INT:
			/* DO NOTHING*/
			break;
		case MR2INT:
			/* DO NOTHING*/
			break;
		case MR3INT:
			/* DO NOTHING*/
			break;
		case CAP0INT:
			/* DO NOTHING*/
			break;
		case CAP1INT:
			/* DO NOTHING*/
			break;
		}
}

/**
  * @brief  Frequency Calculator interrupt handler for TIMER2
  * @retval none
  */
void vFreqCalculator(uint32_t interruptsource)
{

#if TESTING_TIMER2HANDLER
		static uint8_t ledstate=LED_OFF;
#endif
		switch (interruptsource) {
#if TESTING_TIMER2HANDLER
			if (ledstate == LED_OFF)
				vLedOn(LED_2);
			else if (ledstate == LED_ON)
				vLedOff(LED_2);
			ledstate ^= LED_ON;
#endif
			/* Timer1 TC will be holding the number external pulses */
			measuredfrequency= LPC_TIM1->TC;
			vTimerControlReset(TIMER1);
			vTimerControl(TIMER1,TIMER_COUNTER_ENABLE);
			uartprintflag=1;
			break;
		case MR1INT:
			/* DO NOTHING*/
			break;
		case MR2INT:
			/* DO NOTHING*/
			break;
		case MR3INT:
			/* DO NOTHING*/
			break;
		case CAP0INT:
			/* DO NOTHING*/
			break;
		case CAP1INT:
			/* DO NOTHING*/
			break;
		}
}


/**
  * @brief  Initialize all the hardware connected
  * @retval none
  */
void vAppHardwareInit(void)
{
	vPinmuxInitialize();
	vLedInitialize();

	vUARTInitialize(UART0,UART_0,BAUDRATE_9600);
	/* Attaching the vUARTCallbackHandler function when interrupt occurs in UART0 */
	//vUARTIntAttachCallback(UART_0,vUARTCallbackHandler);

	vUARTInterruptDisable(UART0,THRE_INTERRUPT);
	vUARTInterruptDisable(UART0,RBR_INTERRUPT);

	/********************Configuring TIMER0 for Clock Generation**************/
	/* Basic configurations of Timer */
	vTimerInitialize(TIMER_0);
	/* Configuring the timer in Timer Mode */
	vTimerCountControl(TIMER0,TIMER_MODE);
	/* Setting the prescalar for making TC count every 1ms */
	vTimerPrescalarSel(TIMER0,PRESCALAR_MS);
	/* Load the respective match registers with required delay in ms */
	vTimerLoadMatchRegister(TIMER0,MATCH0,100);
	/* Enabling to Reset the timer when TC matches with MR0 register value */
	vTimerMatchReset(TIMER0,MATCH0);
	/* Call back registration of vClockGenerator with Timer0 IRQ Handler */
	vTimerIntAttachCallback(TIMER_0,vClockGenerator);
	/* Enabling interrupt for all match register 0 */
	vTimerInterruptEnable(TIMER0,MATCH0);
	/* Enabling the Timer0 interrupt in NVIC */
	NVIC_EnableIRQ(TIMER0_IRQn);
	/********************************************************************/

	/********************Configuring TIMER2 for Frequency Calculation*********/
	/* Basic configurations of Timer */
	vTimerInitialize(TIMER_2);
	/* Configuring the timer in Timer Mode */
	vTimerCountControl(TIMER2, TIMER_MODE);
	/* Setting the prescalar for making TC count every 1ms */
	vTimerPrescalarSel(TIMER2, PRESCALAR_MS);
	/* Load the respective match registers with required delay in ms */
	vTimerLoadMatchRegister(TIMER2, MATCH0, 1000);
	/* Enabling to Reset the timer when TC matches with MR0 register value */
	vTimerMatchReset(TIMER2, MATCH0);
	/* Call back registration of vFreqCalculator with Timer2 IRQ Handler */
	vTimerIntAttachCallback(TIMER_2, vFreqCalculator);
	/* Enabling interrupt for all match register 0 */
	vTimerInterruptEnable(TIMER2, MATCH0);
	/* Enabling the Timer2 interrupt in NVIC */
	NVIC_EnableIRQ(TIMER2_IRQn);
	/********************************************************************/

	/*********Configuring TIMER1 for counting external clock pulses***********/
	vTimerInitialize(TIMER_1);
	vTimerCountControl(TIMER1, TC_INCREMENT_FALLING_EDGE);
	/* We have configured the CAP1_1 in pinmux for getting the external pulses */
	vTimerCountControlInpSel(TIMER1, CAPn_1);
	/* No scaling as we are going to measure only small frequencies */
	vTimerPrescalarSel(TIMER1, 0);
	/***************************************************************************************/

	/* Starting the clock generation */
	vTimerControlReset(TIMER0);
	vTimerControl(TIMER0, TIMER_COUNTER_ENABLE);

	/* Starting the frequency measurement */
	vTimerControlReset(TIMER2);
	vTimerControl(TIMER2, TIMER_COUNTER_ENABLE);

	/* Start counting the external clock pulses */
	vTimerControlReset(TIMER1);
	vTimerControl(TIMER1, TIMER_COUNTER_ENABLE);
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Variable declarations and initializations */

  /* MCU Configuration--------------------------------------------------------*/
  /* Initialize all configured peripherals */
   vAppHardwareInit();

   vUARTPutStringBlocking(UART0,"External clock frequency measurement\r\n");

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  // for(;;)
  {

		if (uartprintflag) {
			vUARTPrintfBlocking(UART0, "Frequency=%d\r\n",measuredfrequency);
			uartprintflag = 0;
		}

  }
  /* End of Application entry point */
}



