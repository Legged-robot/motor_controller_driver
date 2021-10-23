/*
 * calibration.h
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "position_sensor.h"
#include "foc.h"

#define V_CAL 	1.0f							// Calibration voltage
//#define I_CAL	5.0f
#define ENC_FULL_MECH_ROTATION_TH 	120			// ENC thershold for full mechanical rotation in ticks
#define W_CAL 	8.0f							// Calibration speed in rad/s
#define T1		2.0f							// Cal settling period
#define PPAIRS_MAX 64
#define SAMPLES_PER_PPAIR N_LUT
#define N_CAL SAMPLES_PER_PPAIR*PPAIRS_MAX		// Calibration lookup table maximum length, so I don't have to deal with dynamic allocation based on number of pole pairs

typedef struct{
	uint8_t ppairs;									// number of pole pairs measured
	int offset;								// electrical zero position in counts
	float theta_ref;								// reference angle used for calibration
	int start_count;								// loop count at cal start
	uint8_t started;								// has cal started or not?
	float time;										// cal time
	float next_time_ref;							// cal time
	int enc_start_count;							// cal start encoder count
	int ezero;
	uint8_t phase_order;							// phase order correct (0) or swapped (1)
	uint8_t done_ordering, done_cal, 
			done_ppair_detect, done_rl, done_ezero;							// flags for different cals finished
	uint16_t sample_count;							// keep track of how many samples taken
	float next_sample_time;							// time to take next sample
	int error_arr[N_CAL];
	EncoderStruct encoder_p;						// Position reference used for calibration

} CalStruct;

void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal, int loop_count);
void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal,
		int loop_count);
void set_ezero(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count);
void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct *cal,  int loop_count);

//extern int *error_array;
//extern int *lut_array;

#endif /* INC_CALIBRATION_H_ */
