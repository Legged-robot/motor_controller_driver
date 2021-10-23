/*
 * calibration.c
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */


#include "calibration.h"
#include "hw_config.h"
#include "user_config.h"
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "math_ops.h"


void order_phases(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	/* Checks phase order, to ensure that positive Q current produces
	   torque in the positive direction wrt the position sensor */
	PHASE_ORDER = 0;
	
	if(!cal->started){
		printf("Checking phase sign, pole pairs\r\n");
		cal->started = 1;
		cal->start_count = loop_count;
		cal->next_time_ref = T1+2.0f*PI_F/W_CAL;
		cal->ppairs = 1; //at least one electrical revolution will be performed (at least one ppair is surely available)
	}
	cal->time = (float)(loop_count - cal->start_count)*DT;
	if (cal->time<0){
		printf("order_phases: cal->time<0!!! WARNING!!! \r\n");
	}

    if(cal->time < T1){
        // Set voltage angle to zero, wait for rotor position to settle
        cal->theta_ref = 0;//W_CAL*cal->time;
        cal->encoder_p.elec_angle = cal->theta_ref;
        cal->encoder_p.elec_velocity = 0;
        controller->i_d_des = I_CAL;
        controller->i_q_des = 0.0f;
        commutate(controller, &cal->encoder_p);
    	cal->enc_start_count = encoder->raw;
    	return;
    }
    else if(cal->time < cal->next_time_ref){
    	// Rotate voltage vector through one electrical cycle
    	cal->theta_ref = W_CAL*(cal->time-T1);
    	cal->encoder_p.elec_angle = cal->theta_ref;
		commutate(controller, &cal->encoder_p);
    	return;
    }
	else if (fabsf(encoder->raw-cal->enc_start_count)>ENC_FULL_MECH_ROTATION_TH){
		// Motor still didnt complete one mechanical revolution
		cal->next_time_ref += 2.0f*PI_F/W_CAL;			// Request one more electrical cycle
		cal->ppairs++;									// One more ppair is present
		if (!cal->done_ordering){
			// Finished first elecrical cycle, perform phase order check
			int enc_end_count = encoder->raw;
			if(fabs(cal->enc_start_count - enc_end_count) > ENC_CPR/4){	// Assumption: motor has more than 4 ppairs
				// overflow of the counter
				float correction = cal->enc_start_count > enc_end_count ? ENC_CPR : -ENC_CPR;
				enc_end_count += correction;
			}

			if(cal->enc_start_count < enc_end_count){
				cal->phase_order = 0;
			}
			else{
				cal->phase_order = 1;
			}
			cal->done_ordering = 1;
		}

		//continue with commutation
		cal->theta_ref = W_CAL*(cal->time-T1);
		cal->encoder_p.elec_angle = cal->theta_ref;
		commutate(controller, &cal->encoder_p);
		return;
	}
	else if( (fabs(controller->i_q_filt)>1.0f) || (fabs(controller->i_d_filt)>1.0f) ){
		controller->i_d_des = 0.0f;
		commutate(controller, &cal->encoder_p);
		return;
	}

	//motor completed one mechanical revolution
    reset_foc(controller);

	
	// cal->ppairs = round(2.0f*PI_F/fabsf(enc_end_count-cal->enc_start_count)); //TODO: if sensor is not calibrated, not a valid reference to calculate pparis!
	//	cal->ppairs = 14;	//TODO: change to right value

	if(cal->phase_order == 0){
		printf("Phase order correct\r\n");
	}
	else{
		printf("Swapping phase sign\r\n");
	}
    printf("Pole Pairs: %d\r\n", cal->ppairs);
    // printf("Start: %.3f   End: %.3f\r\n", cal->enc_start_count, enc_end_count);
    PHASE_ORDER = cal->phase_order;
    PPAIRS = (float)cal->ppairs;
	cal->done_ppair_detect = 1;
    cal->started = 0;
    
}

void calibrate_encoder(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	/* Calibrates e-zero and encoder nonliearity */

	if(!cal->started){
			printf("Starting offset cal and linearization\r\n");
			cal->started = 1;
			cal->start_count = loop_count;
			cal->next_sample_time = T1;
			cal->sample_count = 0;
		}

	cal->time = (float)(loop_count - cal->start_count)*DT;
	if (cal->time<0){
		printf("calibrate_encoder: cal->time<0!!! WARNING!!! \r\n");
	}
    if(cal->time < T1){
        // Set voltage angle to zero, wait for rotor position to settle
        cal->theta_ref = 0;//W_CAL*cal->time;
        cal->encoder_p.elec_angle = cal->theta_ref;
        cal->encoder_p.elec_velocity = 0;
        controller->i_d_des = I_CAL;
        controller->i_q_des = 0.0f;
        commutate(controller, &cal->encoder_p);

    	cal->next_sample_time = cal->time;
    	return;
    }
    else if (cal->time < T1+2.0f*PI_F*PPAIRS/W_CAL){
    	// rotate voltage vector through one mechanical rotation in the positive direction
		cal->theta_ref = W_CAL*(cal->time-T1);
		cal->encoder_p.elec_angle = cal->theta_ref;
		commutate(controller, &cal->encoder_p);

		// sample SAMPLES_PER_PPAIR times per pole-pair
		if(cal->time > cal->next_sample_time){
			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
			int error = encoder->raw - count_ref;//- encoder->raw;
			cal->error_arr[cal->sample_count] = error + ENC_CPR*(error<0);
			printf("%d %d %d %.3f\r\n", cal->sample_count, count_ref, cal->error_arr[cal->sample_count], cal->theta_ref);
			cal->next_sample_time += 2.0f*PI_F/(W_CAL*SAMPLES_PER_PPAIR);
			if(cal->sample_count == PPAIRS*SAMPLES_PER_PPAIR-1){
				return;
			}
			cal->sample_count++;

		}
		return;
    }
	else if (cal->time < T1+4.0f*PI_F*PPAIRS/W_CAL){
		// rotate voltage vector through one mechanical rotation in the negative direction
		cal->theta_ref = W_CAL*(4.0f*PI_F*PPAIRS/W_CAL + T1 - cal->time);
		controller->i_d_des = I_CAL;
		controller->i_q_des = 0.0f;
		cal->encoder_p.elec_angle = cal->theta_ref;
		commutate(controller, &cal->encoder_p);

		// sample SAMPLES_PER_PPAIR times per pole-pair
		if((cal->time > cal->next_sample_time)&&(cal->sample_count>0)){
			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
			int error = encoder->raw - count_ref;// - encoder->raw;
			error = error + ENC_CPR*(error<0);

			cal->error_arr[cal->sample_count] = (cal->error_arr[cal->sample_count] + error)/2;
			printf("%d %d %d %.3f\r\n", cal->sample_count, count_ref, cal->error_arr[cal->sample_count], cal->theta_ref);
			cal->sample_count--;
			cal->next_sample_time += 2.0f*PI_F/(W_CAL*SAMPLES_PER_PPAIR);
		}
		return;
    }
	else if( (fabs(controller->i_q_filt)>1.0f) || (fabs(controller->i_d_filt)>1.0f) ){
		controller->i_d_des = 0.0f;
		commutate(controller, &cal->encoder_p);
		return;
	}

	reset_foc(controller);

    // Calculate average offset
    int zero_mean = 0;
	for(int i = 0; i<((int)PPAIRS*SAMPLES_PER_PPAIR); i++){
		zero_mean += cal->error_arr[i];
	}
	zero_mean = zero_mean/(SAMPLES_PER_PPAIR*PPAIRS);

	// Moving average to filter out cogging ripple

	int window = SAMPLES_PER_PPAIR;
	int lut_offset = (ENC_CPR-cal->error_arr[0])*N_LUT/ENC_CPR;
	for(int i = 0; i<N_LUT; i++){
			int moving_avg = 0;
			for(int j = (-window)/2; j<(window)/2; j++){
				int index = i*PPAIRS*SAMPLES_PER_PPAIR/N_LUT + j;
				if(index<0){index += (SAMPLES_PER_PPAIR*PPAIRS);}
				else if(index>(SAMPLES_PER_PPAIR*PPAIRS-1)){index -= (SAMPLES_PER_PPAIR*PPAIRS);}
				moving_avg += cal->error_arr[index];
			}
			moving_avg = moving_avg/window;
			int lut_index = lut_offset + i;
			if(lut_index>(N_LUT-1)){lut_index -= N_LUT;}
			cal->encoder_p.offset_lut[lut_index] = moving_avg - zero_mean;
			printf("%d  %d\r\n", lut_index, cal->encoder_p.offset_lut[lut_index]);

		}
		

	cal->started = 0;
	cal->done_cal = 1;
}
void set_ezero(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	/* Calibrates e-zero and encoder nonliearity */

	if(!cal->started){
			printf("Starting ezero determination procedure\r\n");
			cal->started = 1;
			cal->start_count = loop_count;
			cal->sample_count = 0;
		}

	cal->time = (float)(loop_count - cal->start_count)*DT;
	if (cal->time<0){
		printf("set_ezero: cal->time<0!!! WARNING!!! \r\n");
	}
    if(cal->time < T1){
        // Set voltage angle to zero, wait for rotor position to settle
        cal->theta_ref = 0;//W_CAL*cal->time;
        cal->encoder_p.elec_angle = cal->theta_ref;
        cal->encoder_p.elec_velocity = 0;
        controller->i_d_des = I_CAL;
        controller->i_q_des = 0.0f;
        commutate(controller, &cal->encoder_p);
		
    	return;
    }
    else if (!cal->done_ezero){
		ps_sample(&cal->encoder_p, DT, PS_POLL_FOR_DATA);
    	cal->done_ezero = 1;
    }

    if( (fabs(controller->i_q_filt)>1.0f) || (fabs(controller->i_d_filt)>1.0f) ){
		controller->i_d_des = 0.0f;
		commutate(controller, &cal->encoder_p);
		return;
	}


	reset_foc(controller);

	cal->ezero = cal->encoder_p.count;
	cal->started = 0;
}

void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){

}
