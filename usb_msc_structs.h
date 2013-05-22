/*
 * usb_msc_structs.h
 *
 *  Created on: 22/05/2013
 *      Author: Richard
 */

#ifndef USB_MSC_STRUCTS_H_
#define USB_MSC_STRUCTS_H_

//*****************************************************************************
//
// The mass storage class device structure.
//
//*****************************************************************************
extern const tUSBDMSCDevice g_sMSCDevice;

//*****************************************************************************
//
// The externally provided mass storage class event call back function.
//
//*****************************************************************************
extern unsigned long USBDMSCEventCallback(void *pvCBData, unsigned long ulEvent,
                                       unsigned long ulMsgParam,
                                       void *pvMsgData);

#endif /* USB_MSC_STRUCTS_H_ */
