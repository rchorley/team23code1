#define PART_LM4F120H5QR

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


//*****************************************************************************
//
// GLOBALS AND #DEFINES
//
//*****************************************************************************

static FATFS fs;
static FIL configFile;
static BYTE readBuffer[1250];
//int16_t audioTest[900];
static UINT bw, br;

#define PI 						3.14159265359




// THIS IS WHERE THE CIRCULAR BUFFER/BUTTON  GLOBALS ARE

int16_t cBuff[1323]; // circular buffer for reading SD audio files
int16_t *startPtr = &cBuff[0];
int16_t *readPtr;
int16_t *writePtr;

// arrays to hold information of buttons (maybe int8_t to save on memory)

int8_t latchHold[16]; // which buttons have latch/hold functionality
int8_t playing[16]; // which buttons are playing samples
int8_t looping[16]; // which buttons are looping - work this out later
int8_t pressed[16]; // which buttons are pressed



//*****************************************************************************
//
// The number of ticks to wait before falling back to the idle state.  Since
// the tick rate is 100Hz this is approximately 3 seconds.
//
//*****************************************************************************
#define USBMSC_ACTIVITY_TIMEOUT 300

//*****************************************************************************
//
// This enumeration holds the various states that the device can be in during
// normal operation.
//
//*****************************************************************************
volatile enum
{
    //
    // Unconfigured.
    //
    MSC_DEV_DISCONNECTED,

    //
    // Connected but not yet fully enumerated.
    //
    MSC_DEV_CONNECTED,

    //
    // Connected and fully enumerated but not currently handling a command.
    //
    MSC_DEV_IDLE,

    //
    // Currently reading the SD card.
    //
    MSC_DEV_READ,

    //
    // Currently writing the SD card.
    //
    MSC_DEV_WRITE,
}
g_eMSCState;

//*****************************************************************************
//
// The Flags that handle updates to the status area to avoid drawing when no
// updates are required..
//
//*****************************************************************************
#define FLAG_UPDATE_STATUS      1
static unsigned long g_ulFlags;
static unsigned long g_ulIdleTimeout;

//******************************************************************************
//
// The DMA control structure table.
//
//******************************************************************************
#ifdef ewarm
#pragma data_alignment=1024
tDMAControlTable sDMAControlTable[64];
#elif defined(ccs)
#pragma DATA_ALIGN(sDMAControlTable, 1024)
tDMAControlTable sDMAControlTable[64];
#else
tDMAControlTable sDMAControlTable[64] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
RxHandler(void *pvCBData, unsigned long ulEvent,
               unsigned long ulMsgValue, void *pvMsgData)
{
    return(0);
}

//*****************************************************************************
//
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long
TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          void *pvMsgData)
{
    return(0);
}


//*****************************************************************************
//
// This function is the call back notification function provided to the USB
// library's mass storage class.
//
//*****************************************************************************
unsigned long
USBDMSCEventCallback(void *pvCBData, unsigned long ulEvent,
                     unsigned long ulMsgParam, void *pvMsgData)
{
    //
    // Reset the time out every time an event occurs.
    //
    g_ulIdleTimeout = USBMSC_ACTIVITY_TIMEOUT;

    switch(ulEvent)
    {
        //
        // Writing to the device.
        //
        case USBD_MSC_EVENT_WRITING:
        {
            //
            // Only update if this is a change.
            //
            if(g_eMSCState != MSC_DEV_WRITE)
            {
                //
                // Go to the write state.
                //
                g_eMSCState = MSC_DEV_WRITE;

                //
                // Cause the main loop to update the screen.
                //
                g_ulFlags |= FLAG_UPDATE_STATUS;
            }

            break;
        }

        //
        // Reading from the device.
        //
        case USBD_MSC_EVENT_READING:
        {
            //
            // Only update if this is a change.
            //
            if(g_eMSCState != MSC_DEV_READ)
            {
                //
                // Go to the read state.
                //
                g_eMSCState = MSC_DEV_READ;

                //
                // Cause the main loop to update the screen.
                //
                g_ulFlags |= FLAG_UPDATE_STATUS;
            }

            break;
        }
        //
        // The USB host has disconnected from the device.
        //
        case USB_EVENT_DISCONNECTED:
        {
            //
            // Go to the disconnected state.
            //
            g_eMSCState = MSC_DEV_DISCONNECTED;

            //
            // Cause the main loop to update the screen.
            //
            g_ulFlags |= FLAG_UPDATE_STATUS;

            break;
        }
        //
        // The USB host has connected to the device.
        //
        case USB_EVENT_CONNECTED:
        {
            //
            // Go to the idle state to wait for read/writes.
            //
            g_eMSCState = MSC_DEV_IDLE;

            break;
        }
        case USBD_MSC_EVENT_IDLE:
        default:
        {
            break;
        }
    }

    return(0);
}

//*****************************************************************************
//
// This is the handler for this SysTick interrupt.  FatFs requires a timer tick
// every 10 ms for internal timing purposes.
//
//*****************************************************************************
void
SysTickHandler(void)
{
    //
    // Call the FatFs tick timer.
    //
    disk_timerproc();

    if(g_ulIdleTimeout != 0)
    {
        g_ulIdleTimeout--;
    }
}

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
	// enable SSI0 for DAC
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// configure DAC SSI pins
	GPIOPinConfigure(GPIO_PA2_SSI0CLK); // jtag 2.10
	GPIOPinConfigure(GPIO_PA3_SSI0FSS); // jtag 2.09
	GPIOPinConfigure(GPIO_PA5_SSI0TX); // jtag 1.08
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_3 | GPIO_PIN_2);

	// Clock speed for DAC
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
	                       SSI_MODE_MASTER, 1000000, 16);

	// Enable SSI
	SSIEnable(SSI0_BASE);
}

void InitADC(void)
{
    // The ADC0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH1 | ADC_CTL_IE |
                                 ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 0);

    ADCIntClear(ADC0_BASE, 0);

    ADCIntEnable(ADC0_BASE, 0);

    IntEnable(INT_ADC0SS0);
}

void InitTimers(void)
{
	unsigned long ulDACPeriod, ulPktPeriod;

    // Enable the timers used
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // dac timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // packet of audio

	// Configure the 32-bit periodic timers.
	TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);

	ulDACPeriod = (SysCtlClockGet() / 44100);// 44.1 kHz for DAC samples
	ulPktPeriod = (SysCtlClockGet() / 100); // 10 ms timer for audio packets

	TimerLoadSet(TIMER0_BASE, TIMER_A, ulDACPeriod - 1);
	TimerLoadSet(TIMER1_BASE, TIMER_A, ulPktPeriod);

	// Setup the interrupts for the timer timeouts.
	IntEnable(INT_TIMER0A);
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// Enable processor interrupts.
	IntMasterEnable();

	// Enable the timers.
	TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_A);
}

void DACWrite(uint16_t data)
{
	uint16_t spidata, data2;

	data2 = (data & 0x0FFF);

	spidata = (data2 | 0x3000); // 0x3000 = command before 12 bits of data

	SSIDataPut(SSI0_BASE, spidata);

	while(SSIBusy(SSI0_BASE))
	{
	}
}

void SDwrite(void)
{
    f_open(&configFile, "config4.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS);

    f_write(&configFile, "configure stuff\n", 16, &bw);

    f_close(&configFile);
}

void SDread(void)
{
	int i;
	int sample;
//	UINT length = sizeof(readBuffer);
	UINT length = sizeof(audioTest);
	uint16_t DACBuff[];
	char * tempStr;

	f_open(&configFile, "ping.wav", FA_READ);

	for(sample = 0; sample < length)

	f_read(&configFile, audioTest, length, &br);

	f_close(&configFile);

//	for(i = 0; i < length; i++) {
//		if(readBuffer[i] != '\n') {
//			strcat(tempStr, readBuffer[i]);
//		} else {
//			DACBuff[sample] = (uint16_t)atoi(tempStr);
//			sample++;
//		}
//		UARTprintf("%c", readBuffer[i]);
//	}
}

void generate_sine_wave(int freq)
{
	uint16_t testInput[441];
	int i;
	for (i = 0; i <= 441; i++) {
		testInput[i] = floor(2048*(sin(freq * (2 * PI) * i / 44100))+2048);
		UARTprintf("%d\n", testInput[i]);
	}
}

// function to read from keypad, deal with LEDs, and update functionality arrays
//void poll_buttons(void)
//{
//
//}

//*****************************************************************************
//
// This is the main loop that runs the application.
//
//*****************************************************************************
int
main(void)
{
    unsigned long ulRetcode;

    ROM_FPULazyStackingEnable();

    //
    // Set the system clock to run at MHz from the PLL.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Configure SysTick for a 100Hz interrupt.  The FatFs driver wants a 10 ms
    // tick.
    //
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / 100);
    ROM_SysTickEnable();
    ROM_SysTickIntEnable();

    //
    // Enable Interrupts
    //
    ROM_IntMasterEnable();

    //
    // Initialise UART
    //
    InitUART();


    //
    // Initialise timers
    //
    InitTimers();


    //
    // Configure and enable uDMA
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlDelay(10);
    ROM_uDMAControlBaseSet(&sDMAControlTable[0]);
    ROM_uDMAEnable();



    // Initialize the idle timeout and reset all flags.
    //
    g_ulIdleTimeout = 0;
    g_ulFlags = 0;

    //
    // Initialize the state to idle.
    //
    g_eMSCState = MSC_DEV_DISCONNECTED;

    //
    // Draw the status bar and set it to idle.
    //
   // UpdateStatus("Disconnected", 1);

    //
    // Enable the USB controller.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);

    //
    // Set the USB pins to be controlled by the USB controller.

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //
    // Set the USB stack mode to Device mode with VBUS monitoring.
    //
    USBStackModeSet(0, USB_MODE_DEVICE, 0);

    //
    // Pass our device information to the USB library and place the device
    // on the bus.
    //
    USBDMSCInit(0, (tUSBDMSCDevice *)&g_sMSCDevice);

    //
    // Determine whether or not an SDCard is installed.  If not, print a
    // warning and have the user install one and restart.
    //
    ulRetcode = disk_initialize(0);

    // register work area
    f_mount(0, &fs);

//    UARTprintf("writing\n");
//    SDwrite();

//    UARTprintf("reading\n");
    SDread();
//    generate_sine_wave(1000);
    while(1){
    	;
    }
}

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
//
// This is where the magic happens - reading samples from SD and polling
// keypad buttons. Processing these packets within a circular buffer.
//
//*****************************************************************************

// interrupt handler to fire at frequency 44.1 kHz for samples
void SampleIntHandler(void)
{
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	DACWrite(&readPtr);
	readPtr++;
//	writePtr++;

	if(readPtr == startPtr + 1323) {
		readPtr = startPtr;
	}
}

// interrupt handler to grab packets every 10 ms
void PacketIntHandler(void)
{
	int16_t pkt1[441];
	int16_t pkt2[441];

	// Clear the timer interrupt
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	//poll_buttons(); -> update pressed[] arrays
	for(i = 0; i < 16; i++) {
		if(pressed[i]) {
			if(playing[i]) {
				// read from buffer, wherelast increment pointer (not > sample length)
				//pkt1[0] = packet from buffer
			} else { // pressed and not playing
				//read from sd at 0 (not > sample length)
				//set wherelast pointer
				//pkt1[0] = packet from sd
				//playing[i] = 1;
			}
		} else {
			if(playing[i]) { // not pressed and playing
				if(latch[i]) { // latch
					//if(!samplelength) { // still data to read
						//keep going
					//} else { // end of sample -> finish
						//fill with 0s
					//}
					//pkt1[0] =
				} else { // hold
					// reset wherelast pointer
					//playing[i] = 0;
				}
			}
		}
	}
	// if there are more than one playing, which ones should be playing?
	// convolve (add together and take mean)

	// final samples array
	// apply FX
	cBuff[]
	memcpy()
	writePtr += 441;
//	if(writePtr == startPtr + 1323) {
//		writePtr = 0;
//	}
}

