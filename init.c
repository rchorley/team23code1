/*
 * init.c
 *
 *  Created on: 03/05/2013
 *      Author: Richard
 */
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_hibernate.h"
#include "inc/hw_ints.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"
#include "driverlib/hibernate.h"
#include "driverlib/usb.h"
#include "driverlib/udma.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdmsc.h"

#include "usb_msc_structs.h"
#include "third_party/fatfs/src/diskio.h"
#include "third_party/fatfs/src/ff.h"

void InitUART(void)
{
	// initialise UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioInit(0);
}

void InitDAC(void)
{
//	unsigned long temp;

	// enable SSI1 for DAC
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	// configure DAC SSI pins
	GPIOPinConfigure(GPIO_PB4_SSI2CLK); // jtag 3.03
	GPIOPinConfigure(GPIO_PB5_SSI2FSS); // jtag 3.04
	GPIOPinConfigure(GPIO_PB7_SSI2TX); // jtag 3.06
	GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_7 | GPIO_PIN_5 | GPIO_PIN_4);

	// Clock speed for DAC
	SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
	                       SSI_MODE_MASTER, 1000000, 16);

	// Enable SSI
	SSIEnable(SSI2_BASE);
//	while(SSIDataGetNonBlocking(SSI2_BASE, &temp));
}

void InitADC(void)
{
    // The ADC peripheral(s) must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    SysCtlADCSpeedSet(SYSCTL_ADCSPEED_500KSPS);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_2 | GPIO_PIN_2 |
    		GPIO_PIN_2);

//    ADCSequenceDisable(ADC0_BASE, 1);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH8 | ADC_CTL_IE |
                                 ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 0);

    ADCIntEnable(ADC0_BASE, 0);

    IntEnable(INT_ADC0SS0);

    ADCIntClear(ADC0_BASE, 0);
}

void InitTimers(void)
{
	unsigned long ulDACPeriod, ulPktPeriod;

    // Enable the timers used
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // dac timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // packet of audio

	// Configure the 32-bit periodic timers.
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

	ulDACPeriod = (SysCtlClockGet() / 44100);// 44.1 kHz for DAC samples
	ulPktPeriod = (SysCtlClockGet() / 1); // 10 ms timer for audio packets

	TimerLoadSet(TIMER0_BASE, TIMER_A, ulDACPeriod - 1);
	TimerLoadSet(TIMER1_BASE, TIMER_A, ulPktPeriod);

	// Enable processor interrupts.
//	IntMasterEnable();

	// Enable the timers.
	TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_A);

	// Setup the interrupts for the timer timeouts.

	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	IntEnable(INT_TIMER0A);
	IntEnable(INT_TIMER1A);
}

void InitButtons(void)
{
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 |
			GPIO_PIN_6 | GPIO_PIN_7);

	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
				GPIO_PIN_2 | GPIO_PIN_3);

	// set pull down resistors for the 'reading' port (port D)
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
				GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA,
				GPIO_PIN_TYPE_STD_WPD);

	// set direction of ports
	GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
			GPIO_PIN_2 | GPIO_PIN_3, GPIO_DIR_MODE_IN);

	GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 |
			GPIO_PIN_6 | GPIO_PIN_7, GPIO_DIR_MODE_OUT);

}
