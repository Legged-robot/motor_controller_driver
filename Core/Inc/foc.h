/*
 * foc.h
 *
 *  Created on: Aug 2, 2020
 *      Author: ben
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#include <stdint.h>
#include "position_sensor.h"

// #define CURRENT_THRESHOLD 	0.7f					//Current threshold in Amps
// #define TORQUE_THRESHOLD 	0.2f					//Torque threshold
typedef struct{
    volatile uint8_t adc_data_ready_flag;           // Set once the new data is available
    volatile uint8_t can_fsm_upd_req_flag;			// CAN has sent a command
    volatile char can_command;						// CAN content of the can command
	uint32_t tim_ch_w;								// Terminal W timer channel
    uint16_t adc_data[5];									// Store adc data through DMA updates
    uint16_t adc_ch_i_offset[2];							// Calculated offset of the currentsense channels a and b
    float i_a, i_b, i_c;                                    // Phase currents
    float foc_i_a, foc_i_b, foc_i_c;                                    // Phase currents saved for current foc itterations
    float v_bus, v_bus_filt;                                // DC link voltage
    float theta_mech, theta_elec;                           // Rotor mechanical and electrical angle
    float prev_i_d_error, prev_i_q_error;                   // Previous error values for foc control
    float dtheta_mech, dtheta_elec, dtheta_elec_filt;       // Rotor mechanical and electrical angular velocit
    float i_d, i_q, i_q_filt, i_d_filt;                     // D/Q currents
    float i_mag;											// Current magnitude
    float v_d, v_q;                                         // D/Q voltages
    float dtc_u, dtc_v, dtc_w;                              // Terminal duty cycles
    float v_u, v_v, v_w;                                    // Terminal voltages
    float kp_d, kp_q, ki_d, ki_q, kd_d, kd_q, ki_fw, alpha;               // Current loop gains, current reference filter coefficient
    float d_int, q_int;                                     // Current error integrals
    float i_d_des, i_q_des, i_d_des_filt, i_q_des_filt;     // Current references
    volatile size_t loop_count;                             // Degubbing counter
    volatile size_t timeout;                                // Watchdog counter
    int mode;
    int ovp_flag;                                           // Over-voltage flag
    int oc_flag;											// Over-current flag
    int phase_order;
    volatile union{
    	float commands[5];									// Making this easier to pass around without including foc.h everywhere
    	struct{
    		float p_des, v_des, kp, kd, t_ff;                   // Desired position, velocity, gains, torque
    	};
    };
    float v_max, v_ref, fw_int, int_max;                                    // output voltage magnitude, field-weakening integral
    int otw_flag;                                           // Over-temp warning
    float i_max;											// Maximum current
    float inverter_tab[128];								// Inverter linearization table
    uint8_t invert_dtc;										// Inverter duty cycle inverting/non-inverting
    } ControllerStruct;

typedef struct{
    double temperature;                                     // Estimated temperature
    float temp_measured;									// "Measured" temperature computed from resistance
    float qd_in, qd_out;									// Thermal power in and out
    float resistance;										// Motor resistance
    float k;												// Temperature observer gain
    float trust;											// Temperature observer "trust' (kind of like 1/covariance)
    float delta_t;											// Temperature rise
    }   ObserverStruct;

typedef enum
{
	FOC_CONTINUE_COMMUTATE,
	FOC_NO_REQUEST
}foc_request;

void set_dtc(ControllerStruct *controller);
void calc_analog_data(ControllerStruct *controller);
void abc(float theta, float d, float q, float *a, float *b, float *c);
void dq0(float theta, float a, float b, float c, float *d, float *q);
void svm(float v_max, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w);
void zero_current(ControllerStruct *controller);
void reset_foc(ControllerStruct *controller);
void reset_observer(ObserverStruct *observer);
void init_controller_params(ControllerStruct *controller);
uint8_t safe_exit_commutate(ControllerStruct *controller,  EncoderStruct *encoder, foc_request request);
void commutate(ControllerStruct *controller, EncoderStruct *encoder);
void commutate_d(ControllerStruct *controller, EncoderStruct *encoder, float v_d);
void torque_control(ControllerStruct *controller);
void limit_current_ref (ControllerStruct *controller);
void update_observer(ControllerStruct *controller, ObserverStruct *observer);
void field_weaken(ControllerStruct *controller);
float linearize_dtc(ControllerStruct *controller, float dtc);
void zero_commands(ControllerStruct * controller);


#endif /* INC_FOC_H_ */
