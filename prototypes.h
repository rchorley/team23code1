/*
 * prototypes.h
 *
 *  Created on: 03/05/2013
 *      Author: Richard
 */

#ifndef PROTOTYPES_H_
#define PROTOTYPES_H_

void InitUART(void);
void InitDAC(void);
void InitADC(void);
void InitTimers(void);
void InitButtons(void);
uint8_t SD_read_wav(int8_t button, uint16_t * returnPtr);

#endif /* PROTOTYPES_H_ */
