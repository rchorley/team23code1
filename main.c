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
//static BYTE readBuffer[1250];
uint16_t audioBuffer[441];
static UINT bw, br;

#define PI 						3.14159265359
#define SIZE_OF_PACKET			441

// THIS IS WHERE THE CIRCULAR BUFFER/BUTTON  GLOBALS ARE

uint16_t cBuff[1323]; // circular buffer for reading SD audio files
uint16_t *startPtr = &cBuff[0];
uint16_t *readPtr = &cBuff[0];
uint16_t *writePtr = &cBuff[882];

// arrays to hold information of buttons (maybe int8_t to save on memory)

int8_t latchHold[16]; // which buttons have latch/hold functionality 1 = latch, 0 = hold
int8_t playing[16]; // which buttons are playing samples
int8_t looping[16]; // which buttons are looping - work this out later
int8_t pressed[16]; // which buttons are pressed

//uint32_t sampleLength[16]; // length of each sample
uint32_t whereLast[16]; // how far through the sample we're looking at
const char * fileArr[16][6] = {"00.dat", "01.dat", "02.dat", "03.dat", "04.dat", "05.dat", "06.dat", "07.dat", "08.dat", "09.dat", "10.dat", "11.dat", "12.dat", "13.dat", "14.dat", "15.dat"};


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

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

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
	ulPktPeriod = (SysCtlClockGet() / 100); // 10 ms timer for audio packets

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

void DACWrite(uint16_t data)
{
	uint16_t spidata, data2;

	data2 = (data & 0x0FFF);

	spidata = (data2 | 0x3000); // 0x3000 = command before 12 bits of data

	SSIDataPut(SSI2_BASE, spidata);
//	SSIDataPut(SSI2_BASE, 0x1234);

	while(SSIBusy(SSI2_BASE))
	{
	}
}


uint8_t SD_read_wav(int8_t button, uint16_t * returnPtr)
{
	static FIL sampleFile;
	int k;
	uint8_t fileEnded = 0;
	const char * filename = fileArr[button][0];
	int32_t size;
	uint16_t result;
	UARTprintf("filename = %s\n", filename);

	result = f_open(&sampleFile, filename, FA_OPEN_EXISTING | FA_READ);
	size = sampleFile.fsize;
//	f_lseek(&sampleFile, whereLast[button]);
	UARTprintf("f_size = %d\nresult = %d\n", size, result);

	for(k = 0; k < SIZE_OF_PACKET; k++) {
		uint8_t arr[2];
		int16_t x = 0;

		if((whereLast[button] + 2*k) < size) {
			result = f_read(&sampleFile, &arr[0], 2, &x);
			uint16_t sample = arr[1] | (arr[0] << 8);
			UARTprintf("arr0 = %x, arr1 = %x, x = %d\n", arr[0], arr[1], result);
			*returnPtr = sample;
//			UARTprintf("sample = %d\n", sample);
		} else {
			*returnPtr = 0;
			fileEnded = 1;
		}

		returnPtr++;

//		if(returnPtr == startPtr + 1323){
//			returnPtr = startPtr;
//		}

//		UARTprintf("arr1 = %x\narr0 = %x\nsample = %x\nx = %d\n", arr[1], arr[2], sample, x);
	}
	UARTprintf("%d\n", whereLast[button]);
//	if(fileEnded) {
//		whereLast[button] = 0;
//	} else {
		whereLast[button] += SIZE_OF_PACKET*2;
//	}
//	f_read(&sampleFile, audioBuffer, SIZE_OF_PACKET, NULL);

	f_close(&sampleFile);

	return fileEnded;
}

// function to read from keypad, deal with LEDs, and update functionality arrays
void poll_buttons(void)
{
	int8_t row, col;
	int8_t active = 0;

	//
	unsigned char rowArray[4] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};
	unsigned char colArray[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};
	int8_t button;

	// we need to know which gpio pins to read from and what order to
	// determine which buttons have been pressed.

//	GPIOPinWrite(GPIO_PORTB_BASE, rowArray[row], 0); // set low

	for(row = 0; row < 4; row++) {

		GPIOPinWrite(GPIO_PORTB_BASE, rowArray[row], rowArray[row]); // set high

		for(col = 0; col < 4 ; col++) {

			active = GPIOPinRead(GPIO_PORTD_BASE, colArray[col]); // read

			if(active) { // button has been pressed
				button = 4*row + col - 5;
				pressed[button] = 1;
			}
		}
		GPIOPinWrite(GPIO_PORTB_BASE, rowArray[row], 0); // set low
	}
}

void FXIntHandler(void)
{
//	UARTprintf("adc\n");
	unsigned long ulADC0_Value[4];
//	unsigned long ulADC2_Value[1];
//	unsigned long ulADC3_Value[1];
//	unsigned long ulADC8_Value[1];

	while(!ADCIntStatus(ADC0_BASE, 0, false)) {
	}

	ADCIntClear(ADC0_BASE, 0);

	ADCSequenceDataGet(ADC0_BASE, 0, ulADC0_Value);

	UARTprintf("1a = %5d, 1b = %5d, 2a = %5d, 2b = %5d\r",
			ulADC0_Value[0]/32, ulADC0_Value[1]/32,
			ulADC0_Value[2]/32, ulADC0_Value[3]/32);
}

//*****************************************************************************
//
// This is the main loop that runs the application.
//
//*****************************************************************************
int
main(void)
{
    unsigned long ulRetcode;
    int i = 0;
    int16_t result;

    char * filename = "00.dat";
    FIL testfile;

    ROM_FPULazyStackingEnable();

    for(i = 0; i < 16; i++) {
    	pressed[i] = 0;
    	latchHold[i] = 1;
    	playing[i] = 0;
    	looping[i] = 0;
    	whereLast[i] = 0;
    }

    for(i = 0; i < 1323; i++) {
    	cBuff[i] = 0;
    }

    pressed[0] = 1;
    //
    // Set the system clock to run at MHz from the PLL.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    // register work area
    f_mount(0, &fs);

    //
    // Enable and configure the GPIO port for the LED operation.
    //
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    //
    // Initialise UART
    //
    InitUART();

    InitDAC();

    InitADC();
    //
    // Initialise Buttons
    //
//    InitButtons();

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

//	result = f_open(&testfile, filename, FA_OPEN_EXISTING | FA_READ);
//	UARTprintf("result is %d\n", result);
//	f_close(&testfile);
    //
    // Initialise timers
    //
    InitTimers();


    UARTprintf("starting...\n");
    while(1) {

    	ADCProcessorTrigger(ADC0_BASE, 0);

    	SysCtlDelay(SysCtlClockGet() / 120);
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
//	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
////
//	DACWrite(*readPtr);
//	readPtr++;
//
//	if(readPtr == startPtr + 1323) {
//		readPtr = startPtr; // go back to start of buffer
//	}
}

// interrupt handler to grab packets every 10 ms
void PacketIntHandler(void)
{
	int16_t pkt1[SIZE_OF_PACKET];
	int16_t pkt2[SIZE_OF_PACKET];
	int i;
	uint8_t fileEnded;
	
//	pressed[0] = (pressed[0] + 1)%2;



//	UARTprintf("pressed = %d\n", pressed[0]);
	// Clear the timer interrupt
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	//poll_buttons(); -> update pressed[] arrays

//	if (GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2)) {
//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
//	} else {
//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
//	}
//
//	for(i = 0; i < 16; i++) {
//		if(pressed[i]) {
////			if(playing[i]) {
//			UARTprintf("timer 2\n");
//			SD_read_wav(i, &pkt1[0]);
////				playing[i] = 1;
////			}
//		} else {
//			if(playing[i]) { // not pressed and playing
//				if(latchHold[i]) { // latch
//					fileEnded = SD_read_wav(i, &pkt1[0]);
//					if(fileEnded) {
//						playing[i] = 0;
//					}
//				} else { // hold
//					playing[i] = 0;
//				}
//			}
//		}
//	}
//	if(pressed[0]){
//		UARTprintf("if\n");
//		for(i = 0; i < SIZE_OF_PACKET; i++) {
//			cBuff[i] = pkt1[i];
////			UARTprintf("pkt1 = %d\n", pkt1[i]);
//		}
//	}
//	pressed[0] = 0;
}


//	// if there are more than one playing, which ones should be playing?
//	// convolve (add together and take mean)
//
//	// final samples array
//	// apply FX
////	cBuff[]
////	memcpy()
////	writePtr += SIZE_OF_PACKET;
////	if(writePtr == startPtr + 1323) {
////		writePtr = 0;
////	}
////}

