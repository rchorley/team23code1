/*
 * fx.c
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
#include <stdint.h>
#include "driverlib/uart.h"

// Local includes
#include "fx.h"
#include "sdcard.h"

#include "arm_math.h"
#include "cmsis_ccs.h"

arm_biquad_casd_df1_inst_f32 S1;
arm_biquad_casd_df1_inst_f32 S2;

float32_t inputF32[PKT_SIZE];
float32_t outputF32[PKT_SIZE];

uint8_t numStages = 1;

float32_t iirStateF32a[4];
float32_t iirStateF32b[4];

float32_t pCoeffs1[5]; // filter coefficients
float32_t pCoeffs2[5];

float32_t alpha, omega, a0, a1, a2, b0, b1, b2, c1;
float32_t fc, Q;
float32_t fs = 44100;

float32_t decimate, bitcrush; // decimator/bitcrusher
uint16_t counter1a, counter1b, counter2a, counter2b; // bitwise ko

// what effects to use
extern uint8_t FX1mode, FX2mode;
extern float param1a, param1b, param2a, param2b;



void find_filter_coeffs(uint8_t fxNum, uint8_t filterType, float32_t fc, float32_t Q) {

//	fc = 20.0f+(paramA/128.0f)*8000.0f; // 20 : 8000 Hz
//	Q = (paramB/128.0f); // 0 : 1
//	if(Q == 0.0f) {
//		Q = 0.005;
//	}

	omega = (2.0f * PI * fc) / fs;
	c1 = cos(omega);
	alpha = sin(omega) / (2.0f * Q);

	switch(filterType)  {
		case LOWPASS:

			b0 = (1.0f - c1) / 2.0f;
			b1 = 1.0f - c1;
			b2 = (1.0f - c1) / 2.0f;
			a0 = 1.0f + alpha;
			a1 = -2.0f * c1;
			a2 = 1.0f - alpha;

			break;

		case HIGHPASS:

			b0 = (1.0f + c1) / 2.0f;
			b1 = -1.0f - c1;
			b2 = (1.0f + c1) / 2.0f;
			a0 = 1.0f + alpha;
			a1 = -2.0f * c1;
			a2 = 1.0f - alpha;
			break;

		case BANDPASS:

			b0 = alpha;
			b1 = 0;
			b2 = -1.0f*alpha;
			a0 = 1.0f + alpha;
			a1 = -2.0f * c1;
			a2 = 1.0f - alpha;
			break;

		case NOTCH:

			b0 = 1.0f;
			b1 = -2.0f*c1;
			b2 = 1.0f;
			a0 = 1.0f + alpha;
			a1 = -2.0f * c1;
			a2 = 1.0f - alpha;
			break;
		default:
			break;
	}

	// assign coefficients
	if(fxNum == 1) {
		pCoeffs1[0] = b0 / a0;
		pCoeffs1[1] = b1 / a0;
		pCoeffs1[2] = b2 / a0;
		pCoeffs1[3] = -a1 / a0;
		pCoeffs1[4] = -a2 / a0;
	} else {
		pCoeffs2[0] = b0 / a0;
		pCoeffs2[1] = b1 / a0;
		pCoeffs2[2] = b2 / a0;
		pCoeffs2[3] = -a1 / a0;
		pCoeffs2[4] = -a2 / a0;
	}

}

void fx_init(void) {
	arm_biquad_cascade_df1_init_f32(&S1, numStages, &pCoeffs1[0], &iirStateF32a[0]);
	arm_biquad_cascade_df1_init_f32(&S2, numStages, &pCoeffs2[0], &iirStateF32b[0]);
}

void fx_apply(uint16_t * pkt) {

	int i;
	uint16_t sample;
	int16_t sCentered;
//	float32_t pktFloat;

	//testing
	FX1mode = LOWPASS;
	FX2mode = HIGHPASS;

	if(FX1mode == LOWPASS || FX1mode == HIGHPASS ||
			FX1mode == BANDPASS ||FX1mode == NOTCH) {

		fc = 20.0f+(param1a/128.0f)*8000.0f; // 20 : 8000 Hz
		Q = (param1b/128.0f); // 0 : 1

		if(Q == 0.0f) {
			Q = 0.005;
		}

		find_filter_coeffs(1, FX1mode, fc, Q);

		// convert to floats -1 : 1
		for(i = 0; i < PKT_SIZE; i++) {
			// prevent overflow
			sample = pkt[i];
			sCentered = sample - (1 << 15);
//			pktFloat = (float32_t)(sample >> 2);
			inputF32[i] = sCentered;
		}

		arm_biquad_cascade_df1_f32(&S1, &inputF32[0], &outputF32[0], PKT_SIZE);

		// convert back to uint16_t
		for(i = 0; i < PKT_SIZE; i++) {
			pkt[i] = (uint16_t)(outputF32[i] + (1 << 15));
		}

	} else {
		if(FX1mode == BITWISE_KO) {
			// counters increment in steps of 32 from 0 : 4064
			counter1a = (uint16_t)(param1a)*32; // bitwise AND this counter then,
			counter1b = (uint16_t)(param1b)*32; // bitwise OR this counter with the output sample

			for(i = 0; i < PKT_SIZE; i++) {
				pkt[i] = (pkt[i] & counter1a) ^ counter1b;
			}
		}
		// fx1 is either bitcrush, ko, echo or delay
	}

	if(FX2mode == LOWPASS || FX2mode == HIGHPASS ||
			FX2mode == BANDPASS ||FX2mode == NOTCH) {

		fc = 20.0f+(param2a/128.0f)*8000.0f; // 20 : 8000 Hz
		Q = (param2b/128.0f); // 0 : 1

		if(Q == 0.0f) {
			Q = 0.005;
		}

		find_filter_coeffs(2, FX2mode, fc, Q);

		// convert to floats -1 : 1
		for(i = 0; i < PKT_SIZE; i++) {
			// prevent overflow
			sample = pkt[i];
			sCentered = sample - (1 << 15);
//			pktFloat = (float32_t)(sample >> 2);
			inputF32[i] = sCentered;
		}

		arm_biquad_cascade_df1_f32(&S2, &inputF32[0], &outputF32[0], PKT_SIZE);

		// convert back to uint16_t
		for(i = 0; i < PKT_SIZE; i++) {
			pkt[i] = (uint16_t)(outputF32[i] + (1 << 15));
		}
	} else {
		if(FX2mode == BITWISE_KO) {
			// counters increment in steps of 32 from 0 : 4064
			counter2a = (uint16_t)(param2a)*32; // bitwise AND this counter then,
			counter2b = (uint16_t)(param2b)*32; // bitwise OR this counter with the output sample

			for(i = 0; i < PKT_SIZE; i++) {
				pkt[i] = (pkt[i] & counter2a) ^ counter2b;
			}
		}
		// fx2 is either bitcrush, ko, echo or delay
	}

//	float32_t inputF32[PKT_SIZE];
//	float32_t outputF32[PKT_SIZE];
//
//	uint8_t numStages = 1;
//	float32_t iirStateF32[2];
//	uint16_t sample;
//	float32_t pktFloat;

	// convert to floats -1 : 1
//	for(i = 0; i < PKT_SIZE; i++) {
////		// prevent overflow
//		sample = pkt[i];
//		int16_t sCentered = sample - (1 << 15);
//		pktFloat = (float32_t)(sample >> 2); // div 4
//		inputF32[i] = sCentered; // pktFloat; // (float32_t)((pktFloat/32768.0f) - 0.25f);
//	}
//
//	// apply filter(s)
//	if(filter1) {
//		arm_biquad_cascade_df1_f32(&S1, &inputF32[0], &outputF32[0], PKT_SIZE);
//	}
//	if(filter2) {
//		arm_biquad_cascade_df1_f32(&S2, &inputF32[0], &outputF32[0], PKT_SIZE);
//	}
//
//
//	// convert back to uint16_t
//	for(i = 0; i < PKT_SIZE; i++) {
//		pkt[i] = (uint16_t)(outputF32[i] + (1 << 15));
//	}
//
////	if(filterType = BITWISE_KO) {
////		bitwiseko(&pkt[0]);
////	}
}

//void process_FX(uint8_t FX1mode, uint8_t FX2mode, float32_t param1a,
//		float32_t param1b, float32_t param2a, float32_t param2b) {
//
////	filter1 = 0;
////	filter2 = 0;
////
////	if(FX1mode == LOWPASS || FX1mode == HIGHPASS ||
////			FX1mode == BANDPASS ||FX1mode == NOTCH) {
////		filter1 = 1;
////		find_filter_coeffs(1, FXmode1, param1a, param1b);
////	}
//
//}

//void lowpass(uint16_t * pkt) {
//
//}
//void highpass(uint16_t * pkt) {
//
//}
//void bandpass(uint16_t * pkt) {
//
//}
//void bandstop(uint16_t * pkt) {
//
//}

//void filter(uint16_t * pkt) {
//
//	int i;
//	uint16_t sample;
//
//	// convert to floats -1 : 1
//	for(i = 0; i < PKT_SIZE; i++) {
//
//		// prevent overflow
//		sample = pkt[i];
//		int16_t sCentered = sample - (1 << 15);
//		pktFloat = (float32_t)(sample >> 2); // div 4
//		inputF32[i] = sCentered;
//	}
//
//	// apply filter
//	arm_biquad_cascade_df1_f32(&S, &inputF32[0], &outputF32[0], PKT_SIZE);
//
//	// convert back to uint16_t
//	for(i = 0; i < PKT_SIZE; i++) {
//		pkt[i] = (uint16_t)(outputF32[i] + (1 << 15));
//	}
//}
void delay(uint16_t * pkt) {

}
void echo(uint16_t * pkt) {

}
void decimator(uint16_t * pkt) {

}
void bitcrusher(uint16_t * pkt) {

}
void bitwiseko(uint16_t * pkt) {

//	int i;
//
//	for(i = 0; i < PKT_SIZE; i++) {
//		pkt[i] = (pkt[i] & counter1) ^ counter2;
//	}
}
