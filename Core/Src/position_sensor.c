/*
 * position_sensor.c
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */
#include <stdio.h>
#include <string.h>
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"

void ps_warmup(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
	for(int i = 0; i<n; i++){
		encoder->spi_tx_word = 0x0000;
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
		HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
		while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	}
}

void ps_poll_for_data(EncoderStruct * encoder)
{
	/* SPI read/write */
	encoder->spi_tx_word = ENC_READ_WORD;
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
//	HAL_StatusTypeDef errorcode = HAL_OK;
	HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
//	printf("err_code %d\r\n",errorcode);
	// errorcode = HAL_SPI_Receive(&ENC_SPI, (uint8_t *)encoder->spi_rx_buff, 1, 100);
	// while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	encoder->raw = encoder->spi_rx_word;

	/* Linearization */
	int off_1 = encoder->offset_lut[(encoder->raw)>>9];				// lookup table lower entry
	int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];		// lookup table higher entry
	encoder->off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
}

void ps_sample(EncoderStruct * encoder, float dt, ps_request request)
{
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt */
	HAL_GPIO_WritePin(ADC_INDICATOR, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED, GPIO_PIN_SET);
	/* Shift around previous samples */
	encoder->old_angle = encoder->angle_singleturn;
	/* TODO: This command requires 5us - Better solution would be ring buffer: O(1) */
	memmove(&encoder->angle_multiturn[0], &encoder->angle_multiturn[1], sizeof(*encoder->angle_multiturn)*(N_POS_SAMPLES-1));

	if(request == PS_POLL_FOR_DATA)
	{
		ps_poll_for_data(encoder);
	}

	encoder->count = encoder->raw + encoder->off_interp;
	HAL_GPIO_WritePin(ADC_INDICATOR, GPIO_PIN_RESET);
	/* Real angles in radians */
	encoder->angle_singleturn = ((float)(encoder->count-M_ZERO))/((float)ENC_CPR);
	int int_angle = encoder->angle_singleturn;
	encoder->angle_singleturn = TWO_PI_F*(encoder->angle_singleturn - (float)int_angle);
	//encoder->angle_singleturn = TWO_PI_F*fmodf(((float)(encoder->count-M_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->angle_singleturn = encoder->angle_singleturn<0 ? encoder->angle_singleturn + TWO_PI_F : encoder->angle_singleturn;

	float eoffset_in_ppairs = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
	int ppairs_num = (int)eoffset_in_ppairs;
	encoder->elec_angle = TWO_PI_F*(eoffset_in_ppairs - (float)ppairs_num);
	//encoder->elec_angle = TWO_PI_F*fmodf((encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers
	/* Rollover */
	float angle_diff = encoder->angle_singleturn - encoder->old_angle;
	if(angle_diff > PI_F){encoder->turns--;}
	else if(angle_diff < -PI_F){encoder->turns++;}
	if(!encoder->first_sample){
		encoder->turns = 0;
		encoder->first_sample = 1;
	}



	/* Multi-turn position */
	encoder->angle_multiturn[N_POS_SAMPLES-1] = encoder->angle_singleturn + TWO_PI_F*(float)encoder->turns;

	/* Velocity */
	/*
	// Attempt at a moving least squares.  Wasn't any better
		float m = (float)N_POS_SAMPLES;
		float w = 1.0f/m;
		float q = 12.0f/(m*m*m - m);
		float c1 = 0.0f;
		float ibar = (m - 1.0f)/2.0f;
		for(int i = 0; i<N_POS_SAMPLES; i++){
			c1 += encoder->angle_multiturn[i]*q*(i - ibar);
		}
		encoder->vel2 = -c1/dt;
*/
	//encoder->velocity = vel2
	float vel_angle_diff = encoder->angle_multiturn[N_POS_SAMPLES-1] - encoder->angle_multiturn[0];
	encoder->velocity = (fabsf(vel_angle_diff) < VELOCITY_DETECTION_THRESHOLD ? 0 : vel_angle_diff)/(dt*(float)(N_POS_SAMPLES-1));
	encoder->elec_velocity = encoder->ppairs*encoder->velocity;
	HAL_GPIO_WritePin(LED, GPIO_PIN_RESET);

}

void ps_print(EncoderStruct * encoder, int dt_ms){
	printf("Raw: %d", encoder->raw);
	printf("   Linearized Count: %d", encoder->count);
	printf("   Single Turn: %f", encoder->angle_singleturn);
	printf("   Multiturn: %f", encoder->angle_multiturn[N_POS_SAMPLES-1]);
	printf("   Electrical: %f", encoder->elec_angle);
	printf("   Turns:  %d\r\n", encoder->turns);
	//HAL_Delay(dt_ms);
}
