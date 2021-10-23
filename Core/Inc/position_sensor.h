/*
 * position_sensor.h
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */

#ifndef INC_POSITION_SENSOR_H_
#define INC_POSITION_SENSOR_H_


//#include "structs.h"
#include "spi.h"
#include <stdint.h>

#define N_POS_SAMPLES 20						// Number of position samples to store.  should put this somewhere else...
#define N_LUT 128
#define ZERO_ERROR_OFFSET_COUNTS 5				// Compensate error when setting the new zero position
#define VELOCITY_DETECTION_THRESHOLD 0.0009f	// Dont calculate velocity for angle diff smaller than threshold
typedef struct{
	union{
		uint8_t spi_tx_buff[2];
		uint16_t spi_tx_word;
	};
	union{
		uint8_t spi_rx_buff[2];
		uint16_t spi_rx_word;
	};
	float angle_singleturn, old_angle, angle_multiturn[N_POS_SAMPLES], elec_angle, velocity, elec_velocity, ppairs, vel2;
	float output_angle_multiturn;
	int raw, count, old_count, turns;
	int m_zero, e_zero;
	int offset_lut[N_LUT];
	int off_interp;
	uint8_t first_sample;
} EncoderStruct;


typedef enum
{
    PS_POLL_FOR_DATA,
    PS_NO_DATA_REQUEST
}ps_request;


void ps_warmup(EncoderStruct * encoder, int n);
void ps_poll_for_data(EncoderStruct * encoder);
void ps_sample(EncoderStruct * encoder, float dt, ps_request request);
void ps_print(EncoderStruct * encoder, int dt_ms);

#endif /* INC_POSITION_SENSOR_H_ */
