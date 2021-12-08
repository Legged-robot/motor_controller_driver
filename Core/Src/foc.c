/*
 * foc.c
 *
 *  Created on: Aug 2, 2020
 *      Author: ben
 */

#include "foc.h"
#include "adc.h"
#include "tim.h"
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"
#include "observer.h"

void set_dtc(ControllerStruct *controller)
{

	/* Invert duty cycle if that's how hardware is configured */

	float dtc_u = controller->dtc_u;
	float dtc_v = controller->dtc_v;
	float dtc_w = controller->dtc_w;

	if(INVERT_DTC){
		dtc_u = 1.0f - controller->dtc_u;
		dtc_v = 1.0f - controller->dtc_v;
		dtc_w = 1.0f - controller->dtc_w;

	}
	/* Handle phase order swapping so that voltage/current/torque match encoder direction */
	if(!PHASE_ORDER){
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, ((TIM_PWM.Instance->ARR))*dtc_u);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, ((TIM_PWM.Instance->ARR))*dtc_v);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, ((TIM_PWM.Instance->ARR))*dtc_w);
	}
	else{
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, ((TIM_PWM.Instance->ARR))*dtc_u);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, ((TIM_PWM.Instance->ARR))*dtc_v);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, ((TIM_PWM.Instance->ARR))*dtc_w);
	}
	// dump_state();
}

void calc_analog_data (ControllerStruct *controller)
{
	/* Sampe ADCs */
	/* Handle phase order swapping so that voltage/current/torque match encoder direction */
    // HAL_GPIO_WritePin(DEBUG_LED, GPIO_PIN_SET);

    // By now, new ADC data should be available
    if(!controller->adc_data_ready_flag)
    {
    	while(!controller->adc_data_ready_flag);
    }
    else
    {
    	controller->adc_data_ready_flag = 0; //Indicate that new data is required for next iteration
    }

	if(!PHASE_ORDER){
        controller->i_a = I_SCALE*(float)(controller->adc_ch_i_offset[0] - controller->adc_data[0]);    // Calculate phase currents from ADC readings
        controller->i_b = I_SCALE*(float)(controller->adc_ch_i_offset[1] - controller->adc_data[1]);
	}
	else{
        controller->i_a = I_SCALE*(float)(controller->adc_ch_i_offset[1] - controller->adc_data[1]);    // Calculate phase currents from ADC readings
        controller->i_b = I_SCALE*(float)(controller->adc_ch_i_offset[0] - controller->adc_data[0]);
	}

    controller->i_c = -controller->i_a - controller->i_b;

	controller->v_bus = (float)controller->adc_data[2]*V_SCALE;
	// HAL_GPIO_WritePin(DEBUG_LED, GPIO_PIN_RESET);
}

void abc( float theta, float d, float q, float *a, float *b, float *c)
{
    /* Inverse DQ0 Transform
    Phase current amplitude = lengh of dq vector
    i.e. iq = 1, id = 0, peak phase current of 1 */

    float cf = cos_lut(theta);
    float sf = sin_lut(theta);

    *a = cf*d - sf*q;
    *b = (SQRT3_2*sf-.5f*cf)*d - (-SQRT3_2*cf-.5f*sf)*q;
    *c = (-SQRT3_2*sf-.5f*cf)*d - (SQRT3_2*cf-.5f*sf)*q;
}


void dq0(float theta, float a, float b, float c, float *d, float *q)
{
    /* DQ0 Transform
    Phase current amplitude = lengh of dq vector
    i.e. iq = 1, id = 0, peak phase current of 1*/

    float cf = cos_lut(theta);
    float sf = sin_lut(theta);

    *d = 0.6666667f*(cf*a + (SQRT3_2*sf-.5f*cf)*b + (-SQRT3_2*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-SQRT3_2*cf-.5f*sf)*b - (SQRT3_2*cf-.5f*sf)*c);

}

void svm(float v_max, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w)
{
    /* Space Vector Modulation
     u,v,w amplitude = v_bus for full modulation depth */

    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))*0.5f;
    float dtc_midpoint = .5f*(DTC_MAX+DTC_MIN);
    float dtc_half_range = .5f*(DTC_MAX-DTC_MIN); //must be a positive values

    *dtc_u = fast_fminf(fast_fmaxf(((OVERMODULATION*(u -v_offset)/v_max)*dtc_half_range + dtc_midpoint ), DTC_MIN), DTC_MAX);
    *dtc_v = fast_fminf(fast_fmaxf(((OVERMODULATION*(v -v_offset)/v_max)*dtc_half_range + dtc_midpoint ), DTC_MIN), DTC_MAX);
    *dtc_w = fast_fminf(fast_fmaxf(((OVERMODULATION*(w -v_offset)/v_max)*dtc_half_range + dtc_midpoint ), DTC_MIN), DTC_MAX);

}

void zero_current(ControllerStruct *controller)
{
	/* Measure zero-current ADC offset */

    uint32_t adc_ch_ia_offset = 0;
    uint32_t adc_ch_ib_offset = 0;
    uint32_t n = 1000;
    controller->dtc_u = 0.f;
    controller->dtc_v = 0.f;
    controller->dtc_w = 0.f;
    set_dtc(controller);

    for (int i = 0; i<n; i++){                  // Average n samples
        controller->adc_data_ready_flag = 0;    //Indicate that new data is required for next iteration
		/* Manually start ADC - Possible based on Reference Manual: 13.6
		 * Software source trigger events can be generated ...
		*/
        SET_BIT(ADC_CH_MAIN.Instance->CR2, ADC_CR2_SWSTART);
        /* EOC should be set once conversion from all 3 ADCs is completed.
         * Ref: 13.9.2 Regular simultaneous mode
         * Valid way to check for finish is to monitor flag set by DMA
         * 	*/
        /* HAL can be used but will be slower */
//  	HAL_ADC_Start(&hadc1);
//    	HAL_ADC_PollForConversion(&ADC_CH_MAIN, HAL_MAX_DELAY);
    	calc_analog_data(controller); //waiting for new data flag implemented in the function
    	adc_ch_ia_offset +=  controller->adc_data[0];
    	adc_ch_ib_offset += controller->adc_data[1];
     }
    controller->adc_ch_i_offset[0] = adc_ch_ia_offset/n;
    controller->adc_ch_i_offset[1] = adc_ch_ib_offset/n;

}

void init_controller_params(ControllerStruct *controller)
{

	controller->ki_d = KI_D;
    controller->ki_q = KI_Q;
    controller->kp_d = K_SCALE*I_BW;
    controller->kp_q = K_SCALE*I_BW;
    controller->kd_d = KD_D;
    controller->kd_q = KD_Q;
    // controller->alpha = 1.0f - 1.0f/(1.0f - DT*I_BW*TWO_PI_F);
    // controller->ki_fw = .1f*controller->ki_d;
    controller->phase_order = PHASE_ORDER;
    for(int i = 0; i<128; i++)	// Approximate duty cycle linearization
    {
        controller->inverter_tab[i] = 1.0f + 1.2f*exp(-0.0078125f*i/.032f);
    }
    zero_commands(controller);
}

void reset_foc(ControllerStruct *controller)
{

	TIM_PWM.Instance->CCR3 = ((TIM_PWM.Instance->ARR))*(0.5f);
	TIM_PWM.Instance->CCR1 = ((TIM_PWM.Instance->ARR))*(0.5f);
	TIM_PWM.Instance->CCR2 = ((TIM_PWM.Instance->ARR))*(0.5f);
    controller->i_d_des = 0;
    controller->i_q_des = 0;
    controller->i_d = 0;
    controller->i_q = 0;
    controller->i_d_filt = 0;
    controller->i_q_filt = 0;
    controller->v_bus_filt = 0;
    controller->q_int = 0;
    controller->d_int = 0;
    controller->v_q = 0;
    controller->v_d = 0;
    controller->fw_int = 0;
    controller->otw_flag = 0;

}

void reset_observer(ObserverStruct *observer)
{
/*
    observer->temperature = 25.0f;
    observer->temp_measured = 25.0f;
    //observer->resistance = .1f;
*/
}

void update_observer(ControllerStruct *controller, ObserverStruct *observer)
{
	/*
    /// Update observer estimates ///
    // Resistance observer //
    // Temperature Observer //
    observer->delta_t = (float)observer->temperature - T_AMBIENT;
    float i_sq = controller->i_d*controller->i_d + controller->i_q*controller->i_q;
    observer->q_in = (R_NOMINAL*1.5f)*(1.0f + .00393f*observer->delta_t)*i_sq;
    observer->q_out = observer->delta_t*R_TH;
    observer->temperature += (INV_M_TH*DT)*(observer->q_in-observer->q_out);

    //float r_d = (controller->v_d*(DTC_MAX-DTC_MIN) + SQRT3*controller->dtheta_elec*(L_Q*controller->i_q))/(controller->i_d*SQRT3);
    float r_q = (controller->v_q*(DTC_MAX-DTC_MIN) - SQRT3*controller->dtheta_elec*(L_D*controller->i_d + WB))/(controller->i_q*SQRT3);
    observer->resistance = r_q;//(r_d*controller->i_d + r_q*controller->i_q)/(controller->i_d + controller->i_q); // voltages more accurate at higher duty cycles

    //observer->resistance = controller->v_q/controller->i_q;
    if(isnan(observer->resistance) || isinf(observer->resistance)){observer->resistance = R_NOMINAL;}
    float t_raw = ((T_AMBIENT + ((observer->resistance/R_NOMINAL) - 1.0f)*254.5f));
    if(t_raw > 200.0f){t_raw = 200.0f;}
    else if(t_raw < 0.0f){t_raw = 0.0f;}
    observer->temp_measured = .999f*observer->temp_measured + .001f*t_raw;
    float e = (float)observer->temperature - observer->temp_measured;
    observer->trust = (1.0f - .004f*fminf(abs(controller->dtheta_elec), 250.0f)) * (.01f*(fminf(i_sq, 100.0f)));
    observer->temperature -= observer->trust*.0001f*e;
    //printf("%.3f\n\r", e);

    if(observer->temperature > TEMP_MAX){controller->otw_flag = 1;}
    else{controller->otw_flag = 0;}
    */
}

float linearize_dtc(ControllerStruct *controller, float dtc)
{
    float duty = fast_fmaxf(fast_fminf(fabs(dtc), .999f), 0.0f);;
    int index = (int) (duty*127.0f);
    float val1 = controller->inverter_tab[index];
    float val2 = controller->inverter_tab[index+1];
    return val1 + (val2 - val1)*(duty*128.0f - (float)index);
}

void field_weaken(ControllerStruct *controller)
{
       /// Field Weakening ///

       controller->fw_int += .005f*(controller->v_max - controller->v_ref);
       controller->fw_int = fast_fmaxf(fast_fminf(controller->fw_int, 0.0f), -I_FW_MAX);
       controller->i_d_des = controller->fw_int;
       controller->i_max = I_MAX;
       float q_max = sqrtf(controller->i_max*controller->i_max - controller->i_d_des*controller->i_d_des);
       controller->i_q_des = fast_fmaxf(fast_fminf(controller->i_q_des, q_max), -q_max);

}

uint8_t safe_exit_commutate(ControllerStruct *controller,  EncoderStruct *encoder, foc_request request)
{
	/* Don't stop commutating if there are high currents or FW happening */
	double abs_i_q_filt = fabs(controller->i_q_filt);
	double abs_i_d_filt = fabs(controller->i_d_filt);
	if( (abs_i_q_filt < 1.0f) && (abs_i_d_filt < 1.0f) )
	{
		reset_foc(controller);
		/* Additional safety: wait for current to reduce further after the PWM control reset*/
		if( (abs_i_q_filt < 0.7f) && (abs_i_d_filt < 0.7f) )
		{
			return 1;
		}
	}
	else
	{
		zero_commands(controller);		// Set commands to zero
	}

	/* Request might be issued in cases when explicit
	 * call to commutate is required in order to continue
	 * commutation for further current reduction
	 * */
	if (request == FOC_CONTINUE_COMMUTATE)
	{
		commutate(controller, encoder);
	}
}

void commutate(ControllerStruct *controller, EncoderStruct *encoder)
{
	/* Do Field Oriented Control */
        /* TODO: remove printing of current values for commutations*/
        // printf("i_a: %.2f ib: %.2f i_d: %.2f i_q: %.2f dtheta_elec: %.2f\r\n", controller->i_a, controller->i_b, controller->i_d, controller->i_q, controller->dtheta_elec);

		controller->theta_elec = encoder->elec_angle;
		controller->dtheta_elec = encoder->elec_velocity;
		controller->dtheta_mech = encoder->velocity/GR;
		controller->theta_mech = encoder->angle_multiturn[N_POS_SAMPLES-1]/GR;
        controller->foc_i_a = controller->i_a;
        controller->foc_i_b = controller->i_b;
        controller->foc_i_c = controller->i_c;
       /// Commutation  ///
       dq0(controller->theta_elec, controller->foc_i_a, controller->foc_i_b, controller->foc_i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents - 3.8 us

//       float i_dq = sqrtf(controller->i_d*controller->i_d + controller->i_q*controller->i_q);	// Total current present in the system
//
//       controller->i_d = i_dq < CURRENT_THRESHOLD ? 0 : controller->i_d;	//Introduce current threshold
//       controller->i_q = i_dq < CURRENT_THRESHOLD ? 0 : controller->i_q;
       /* Filtered current */
       controller->i_q_filt = (1.0f-CURRENT_FILT_ALPHA)*controller->i_q_filt + CURRENT_FILT_ALPHA*controller->i_q;	// these aren't used for control but are sometimes nice for debugging
       controller->i_d_filt = (1.0f-CURRENT_FILT_ALPHA)*controller->i_d_filt + CURRENT_FILT_ALPHA*controller->i_d;
       
       /* Filtered bus voltage */
       controller->v_bus_filt = (1.0f-VBUS_FILT_ALPHA)*controller->v_bus_filt + VBUS_FILT_ALPHA*controller->v_bus;

       controller->v_max = OVERMODULATION*controller->v_bus_filt*(DTC_MAX-DTC_MIN)*SQRT1_3; //TODO: remove sqrt(1/3)
       controller->i_max = I_MAX; //I_MAX*(!controller->otw_flag) + I_MAX_CONT*controller->otw_flag;
       controller->int_max = INTEGRAL_MAX_TO_VMAX_RATIO*controller->v_max;

       limit_norm(&controller->i_d_des, &controller->i_q_des, controller->i_max);	// 2.3 us

       /// PI Controller ///
       float i_d_error = controller->i_d_des - controller->i_d;
       float i_q_error = controller->i_q_des - controller->i_q;

       // Calculate decoupling feed-forward voltages //
       float v_d_ff = 0.0f;//-controller->dtheta_elec*L_Q*controller->i_q;
       float v_q_ff = 0.0f;//controller->dtheta_elec*L_D*controller->i_d;

//         float di_d_error = (i_d_error - controller->prev_i_d_error)/(controller->loop_count*DT);
//         float di_q_error = (i_q_error - controller->prev_i_q_error)/(controller->loop_count*DT);
        float di_d_error = 0;
        float di_q_error = 0;
       controller->v_d = controller->kp_d*i_d_error + controller->d_int + controller->kd_d*di_d_error + v_d_ff;
       controller->v_d = fast_fmaxf(fast_fminf(controller->v_d, controller->v_max), -controller->v_max);
       controller->d_int += controller->kp_d*controller->ki_d*i_d_error;	//TODO: zero out integral values for small errors
       controller->d_int = fast_fmaxf(fast_fminf(controller->d_int, controller->int_max), -controller->int_max); // | d_int |<= v_max
//       controller->d_int = 0;

       controller->v_q = controller->kp_q*i_q_error + controller->q_int + controller->kd_q*di_q_error + v_q_ff;
       controller->v_q = fast_fmaxf(fast_fminf(controller->v_q, controller->v_max), -controller->v_max);
       controller->q_int += controller->kp_q*controller->ki_q*i_q_error;
       controller->q_int = fast_fmaxf(fast_fminf(controller->q_int, controller->int_max), -controller->int_max);
//       controller->q_int = 0;

       controller->v_ref = sqrtf(controller->v_d*controller->v_d + controller->v_q*controller->v_q); // used for field weakening

       limit_norm(&controller->v_d, &controller->v_q, controller->v_max);

       abc(controller->theta_elec + 1.5f*DT*controller->dtheta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
       svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation

       set_dtc(controller);
//       controller->loop_count = 0; //reset loop counter
//       controller->prev_i_d_error = i_d_error;
//       controller->prev_i_q_error = i_q_error;

}

/* Apply desired voltage to motor d axis */
void commutate_d(ControllerStruct *controller, EncoderStruct *encoder, float v_d)
{

		controller->theta_elec = encoder->elec_angle;
		controller->dtheta_elec = encoder->elec_velocity;
		controller->v_d = v_d;

       controller->v_max = OVERMODULATION*controller->v_bus*(DTC_MAX-DTC_MIN)*SQRT1_3; //TODO: remove sqrt(1/3)
       controller->v_d = fast_fmaxf(fast_fminf(controller->v_d, controller->v_max), -controller->v_max);
       controller->v_q = 0;

       abc(controller->theta_elec + 1.5f*DT*controller->dtheta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
       svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation

       set_dtc(controller);

}


void torque_control(ControllerStruct *controller)
{

    float ee_torque_des = controller->kp*(controller->p_des - controller->theta_mech) + controller->t_ff + controller->kd*(controller->v_des - controller->dtheta_mech);
    float motor_torque_des = ee_torque_des/(KT*GR);
    // if(fabsf(motor_torque_des) < TORQUE_THRESHOLD){
    // 	controller->i_q_des = 0;
    //     controller->d_int = controller->d_int/1.9;	// Strategy to reduce integral accumulation
    //     controller->q_int = controller->q_int/1.9;
    // }
    // else {
    // 	controller->i_q_des = motor_torque_des;
    // }
    controller->i_q_des = motor_torque_des;
    controller->i_d_des = 0.0f;

}



void zero_commands(ControllerStruct * controller)
{
	controller->t_ff = 0.0f;
	controller->kp = 0.0f;
	controller->kd = 0.0f;
	controller->p_des = 0.0f;
	controller->v_des = 0.0f;
	controller->i_q_des = 0.0f;
	controller->i_d_des = 0.0f;
}
