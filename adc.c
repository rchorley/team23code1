/*
 * adc.c
 *
 *  Created on: 16/05/2013
 *      Author: Coen
 */

// Library includes
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include <stdint.h>

// Local includes
#include "adc.h"



void adc_init(void) {
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
