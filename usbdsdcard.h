/*
 * usbdsdcard.h
 *
 *  Created on: 22/05/2013
 *      Author: Richard
 */

#ifndef USBDSDCARD_H_
#define USBDSDCARD_H_

extern void * USBDMSCStorageOpen(unsigned long ulDrive);
extern void USBDMSCStorageClose(void * pvDrive);
extern unsigned long USBDMSCStorageRead(void * pvDrive, unsigned char *pucData,
                                        unsigned long ulSector,
                                        unsigned long ulNumBlocks);
extern unsigned long USBDMSCStorageWrite(void * pvDrive, unsigned char *pucData,
                                         unsigned long ulSector,
                                         unsigned long ulNumBlocks);
unsigned long USBDMSCStorageNumBlocks(void * pvDrive);

#endif /* USBDSDCARD_H_ */
