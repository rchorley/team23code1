/*
 * fx.h
 *
 *  Created on: 16/05/2013
 *      Author: Coen
 */

#ifndef FX_H_
#define FX_H_

extern void fx_init(void);
extern void find_filter_coeffs(uint8_t FXnum, uint8_t filterType, float fc, float Q);
extern void fx_apply(uint16_t * pkt);
extern void lowpass(uint16_t * pkt);
extern void highpass(uint16_t * pkt);
extern void bandpass(uint16_t * pkt);
extern void bandstop(uint16_t * pkt);
extern void delay(uint16_t * pkt);
extern void echo(uint16_t * pkt);
extern void decimator(uint16_t * pkt);
extern void bitcrusher(uint16_t * pkt);
extern void bitwiseko(uint16_t * pkt);

#define NO_EFFECT			0
#define LOWPASS 			1
#define HIGHPASS 			2
#define BANDPASS 			3
#define NOTCH 				4
#define ECHO				5
#define DECI_BIT_CRUSH		6
#define BITWISE_KO			7
#define DELAY				8


#endif /* FX_H_ */
