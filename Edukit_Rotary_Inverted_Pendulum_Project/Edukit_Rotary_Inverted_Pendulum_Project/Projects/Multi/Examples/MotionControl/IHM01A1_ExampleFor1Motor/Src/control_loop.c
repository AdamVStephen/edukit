
#include "control_loop.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

/*
 * PID Controller with low pass filter operating on derivative component
 */

__INLINE void pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float * current_error,
		float * sample_period, float * Deriv_Filt) {

		float int_term, diff, diff_filt;

	  /* Compute time integral of error by trapezoidal rule */
	  int_term = PID->Ki*(*sample_period)*((*current_error) + PID->state_a[0])/2;

	  /* Compute time derivative of error */
	  diff = PID->Kd*((*current_error) - PID->state_a[0])/(*sample_period);

	  /* Compute first order low pass filter of time derivative */
	  diff_filt = Deriv_Filt[0] * diff
				+ Deriv_Filt[0] * PID->state_a[2]
				- Deriv_Filt[1] * PID->state_a[3];

	  /* Accumulate PID output with Integral, Derivative and Proportional contributions*/

	  PID->control_output = diff_filt + int_term + PID->Kp*(*current_error);

	  /* Update state variables */
	  PID->state_a[1] = PID->state_a[0];
	  PID->state_a[0] = *current_error;
	  PID->state_a[2] = diff;
	  PID->state_a[3] = diff_filt;
	  PID->int_term = int_term;
}

void initialise(){


	/* initialize Integrator Mode time variables */
	apply_acc_start_time = 0;
	clock_int_time = 0;
	clock_int_tick = 0;

	/* Initialize PWM period variables used by step interrupt */
	desired_pwm_period = 0;
	current_pwm_period = 0;
	target_velocity_prescaled = 0;

	/* Initialize default start mode and reporting mode */
	mode_index = 1;
	report_mode = 1;

	/*Initialize serial read variables */
	RxBuffer_ReadIdx = 0;
	RxBuffer_WriteIdx = 0;
	readBytes = 0;

	/*Initialize encoder variables */
	encoder_position = 0;
	encoder_position_down = 0;
	encoder_position_curr = 0;
	encoder_position_prev = 0;
	angle_scale = ENCODER_READ_ANGLE_SCALE;

	/*Initialize rotor control variables */
	rotor_control_target_steps = 0;
	rotor_control_target_steps_curr = 0;
	rotor_control_target_steps_prev = 0;

	/*Initialize rotor plant design transfer function computation variables */
	rotor_control_target_steps_filter_prev_2 = 0.0;
	rotor_control_target_steps_filter_prev_prev_2 = 0.0;
	rotor_control_target_steps_prev_prev = 0.0;

	/* Initialize LQR integral control variables */
	current_error_rotor_integral = 0;

	/*Initialize rotor tracking signal variables */
	enable_rotor_chirp = 0;
	rotor_chirp_start_freq = ROTOR_CHIRP_START_FREQ;
	rotor_chirp_end_freq = ROTOR_CHIRP_END_FREQ;
	rotor_chirp_period = ROTOR_CHIRP_PERIOD;
	enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
	enable_rotor_position_step_response_cycle = ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE;
	disable_mod_sin_rotor_tracking = 0;
	sine_drive_transition = 0;
	mod_sin_amplitude = MOD_SIN_AMPLITUDE;
	rotor_control_sin_amplitude = MOD_SIN_AMPLITUDE;

	/*Initialize sensitivity function selection variables */
	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;
	enable_pendulum_position_impulse_response_cycle = 0;

	/*Initialize user adjustment variables */
	step_size = 0;
	adjust_increment = 0.5;

	/*Initialize adaptive mode state variables */
	mode_transition_state = 0;
	transition_to_adaptive_mode = 0;

	/*Initialize user interactive mode */
	char_mode_select = 0;

	/* Default controller gains */
	proportional = PRIMARY_PROPORTIONAL_MODE_1;
	integral = PRIMARY_INTEGRAL_MODE_1;
	derivative = PRIMARY_DERIVATIVE_MODE_1;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;

	/* Enable State Feedback mode and Integral Action Compensator by default and set
	 * precompensation factor to unity
	 */
	enable_state_feedback = 1;
	integral_compensator_gain = 0;
	feedforward_gain = 1;

}
void main_control(){




	PID_Pend.Kp = proportional * CONTROLLER_GAIN_SCALE;
	PID_Pend.Ki = integral * CONTROLLER_GAIN_SCALE;
	PID_Pend.Kd = derivative * CONTROLLER_GAIN_SCALE;

	PID_Rotor.Kp = rotor_p_gain * CONTROLLER_GAIN_SCALE;
	PID_Rotor.Ki = rotor_i_gain * CONTROLLER_GAIN_SCALE;
	PID_Rotor.Kd = rotor_d_gain * CONTROLLER_GAIN_SCALE;

	PID_Pend.Kp = proportional * CONTROLLER_GAIN_SCALE;
	PID_Pend.Ki = integral * CONTROLLER_GAIN_SCALE;
	PID_Pend.Kd = derivative * CONTROLLER_GAIN_SCALE;

	PID_Rotor.Kp = rotor_p_gain * CONTROLLER_GAIN_SCALE;
	PID_Rotor.Ki = rotor_i_gain * CONTROLLER_GAIN_SCALE;
	PID_Rotor.Kd = rotor_d_gain * CONTROLLER_GAIN_SCALE;

	/*
	 * Initialize Primary and Secondary PID controllers
	 */

	*current_error_steps = 0;
	*current_error_rotor_steps = 0;
	PID_Pend.state_a[0] = 0;
	PID_Pend.state_a[1] = 0;
	PID_Pend.state_a[2] = 0;
	PID_Pend.state_a[3] = 0;
	PID_Pend.int_term = 0;
	PID_Pend.control_output = 0;
	PID_Rotor.state_a[0] = 0;
	PID_Rotor.state_a[1] = 0;
	PID_Rotor.state_a[2] = 0;
	PID_Rotor.state_a[3] = 0;
	PID_Rotor.int_term = 0;
	PID_Rotor.control_output = 0;


	integral_compensator_gain = integral_compensator_gain * CONTROLLER_GAIN_SCALE;

	/* Assign Rotor Plant Design variable values */


	/* Transfer function model of form 1/(s^2 + 2*Damping_Coefficient*Wn*s + Wn^2) */
	if (rotor_damping_coefficient != 0 || rotor_natural_frequency != 0){
		Wn2 = rotor_natural_frequency * rotor_natural_frequency;
		rotor_plant_gain = rotor_plant_gain * Wn2;
		ao = ((2.0F/Tsample)*(2.0F/Tsample) + (2.0F/Tsample)*2.0F*rotor_damping_coefficient*rotor_natural_frequency
				+ rotor_natural_frequency*rotor_natural_frequency);
		c0 = ((2.0F/Tsample)*(2.0F/Tsample)/ao);
		c1 = -2.0F * c0;
		c2 = c0;
		c3 = -(2.0F*rotor_natural_frequency*rotor_natural_frequency - 2.0F*(2.0F/Tsample)*(2.0F/Tsample))/ao;
		c4 = -((2.0F/Tsample)*(2.0F/Tsample) - (2.0F/Tsample)*2.0F*rotor_damping_coefficient*rotor_natural_frequency
				+ rotor_natural_frequency*rotor_natural_frequency)/ao;
	}

	/* Transfer function model of form 1/(s^2 + Wn*s) */
	if (enable_rotor_plant_design == 2){
		IWon_r = 2 / (Wo_r * Tsample);
		iir_0_r = 1 - (1 / (1 + IWon_r));
		iir_1_r = -iir_0_r;
		iir_2_r = (1 / (1 + IWon_r)) * (1 - IWon_r);
	}

	/* Optional Transfer function model of form Wn/(s^3 + Wn*s^2)
	if (enable_rotor_plant_design == 3 && enable_state_feedback == 0){
		  IWon_r = 2 / (Wo_r * Tsample);
		  iir_0_r = 1 / (1 + IWon_r);
		  iir_1_r = iir_0_r;
		  iir_2_r = iir_0_r * (1 - IWon_r);
	}
	*/

	/*

	//Optional display coefficients for rotor plant design transfer function
	sprintf(tmp_string, "\n\rEnable Design: %i iir_0 %0.4f iir_1 %0.4f iir_2 %0.4f\n\r", enable_rotor_plant_design, iir_0_r, iir_1_r, iir_2_r);
	HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);


	//Optional display coefficients for rotor plant design transfer function
	sprintf(tmp_string,
			"\n\ra0 %0.4f c0 %0.4f c1 %0.4f c2 %0.4f c3 %0.4f c4 %0.4f\n\r", ao, c0, c1, c2, c3, c4);
	HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

	 */



	/* Initialize Pendulum PID control state */
	pid_filter_control_execute(&PID_Pend, current_error_steps, sample_period, Deriv_Filt_Pend);

	/* Initialize Rotor PID control state */
	*current_error_rotor_steps = 0;
	pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, sample_period_rotor, Deriv_Filt_Rotor);


	/* Initialize control system variables */

	cycle_count = CYCLE_LIMIT;
	i = 0;
	rotor_position_steps = 0;
	rotor_position_steps_prev = 0;
	rotor_position_filter_steps = 0;
	rotor_position_filter_steps_prev = 0;
	rotor_position_command_steps = 0;
	rotor_position_diff = 0;
	rotor_position_diff_prev = 0;
	rotor_position_diff_filter = 0;
	rotor_position_diff_filter_prev = 0;
	rotor_position_step_polarity = 1;
	encoder_angle_slope_corr_steps = 0;
	rotor_sine_drive = 0;
	sine_drive_transition = 0;

	rotor_mod_control = 1.0;

	enable_adaptive_mode = 0;

	tick_cycle_start = HAL_GetTick();
	tick_cycle_previous = tick_cycle_start;
	tick_cycle_current =  tick_cycle_start;

	enable_cycle_delay_warning = ENABLE_CYCLE_DELAY_WARNING;
	chirp_cycle = 0;
	chirp_dwell_cycle = 0;
	pendulum_position_command_steps = 0;
	impulse_start_index = 0;
	mode_transition_state = 0;
	transition_to_adaptive_mode = 0;
	error_sum_prev = 0;
	error_sum_filter_prev = 0;
	adaptive_state = 4;
	rotor_control_target_steps_prev = 0;
	rotor_position_command_steps_prev = 0;
	rotor_position_command_steps_pf_prev = 0;
	enable_high_speed_sampling = ENABLE_HIGH_SPEED_SAMPLING_MODE;
	slope_prev = 0;
	rotor_track_comb_command = 0;
	noise_rej_signal_prev = 0;
	noise_rej_signal_filter_prev = 0;
	full_sysid_start_index = -1;
	current_cpu_cycle = 0;
	speed_scale = DATA_REPORT_SPEED_SCALE;
	speed_governor = 0;
	encoder_position_offset = 0;
	encoder_position_offset_zero = 0;

	for (m = 0; m < ANGLE_CAL_OFFSET_STEP_COUNT + 1; m++){
		offset_angle[m] = 0;
	}

	/* Initial control state parameter storage */
	float init_r_p_gain = PID_Rotor.Kp;
	float init_r_i_gain = PID_Rotor.Ki;
	float init_r_d_gain = PID_Rotor.Kd;
	float init_p_p_gain = PID_Pend.Kp;
	float init_p_i_gain = PID_Pend.Ki;
	float init_p_d_gain = PID_Pend.Kd;
	int init_enable_state_feedback = enable_state_feedback;
	int init_integral_compensator_gain = integral_compensator_gain;
	int init_feedforward_gain = feedforward_gain;
	int init_enable_state_feedback = enable_state_feedback;
	int init_enable_disturbance_rejection_step = enable_disturbance_rejection_step;
	int init_enable_sensitivity_fnc_step = enable_sensitivity_fnc_step;
	int init_enable_noise_rejection_step = enable_noise_rejection_step;
	int init_enable_rotor_plant_design = enable_rotor_plant_design;
	int init_enable_rotor_plant_gain_design = enable_rotor_plant_gain_design;

	if(select_suspended_mode == 1){
		load_disturbance_sensitivity_scale = 1.0;
	}
	if(select_suspended_mode == 0){
		load_disturbance_sensitivity_scale = LOAD_DISTURBANCE_SENSITIVITY_SCALE;
	}


	steval_edukit_swingup();


}

/*
 * Initiate Pendulum Swing Up with automatic system requiring no user action
 *
 * This system was developed by Markus Dauberschmidt see
 * https://github.com/OevreFlataeker/steval_edukit_swingup
 *
 */
void steval_edukit_swingup(){
	/*
	 * Apply controller parameters for initial operation at completion of
	 * Swing Up
	 */

	PID_Rotor.Kp = 20;
	PID_Rotor.Ki = 10;
	PID_Rotor.Kd = 10;
	PID_Pend.Kp = 300;
	PID_Pend.Ki = 0.0;
	PID_Pend.Kd = 30.0;
	enable_state_feedback = 0;
	integral_compensator_gain = 0;
	feedforward_gain = 1;
	rotor_position_command_steps = 0;
	enable_state_feedback = 0;
	enable_disturbance_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;
	enable_noise_rejection_step = 0;
	enable_rotor_plant_design = 0;
	enable_rotor_plant_gain_design = 0;

	/* Set Torque Current value to 800 mA (normal operation will revert to 400 mA */
	//torq_current_val = MAX_TORQUE_SWING_UP;
	//L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
	L6474_SetAnalogValue_Wrapper();

	sprintf(msg, "Pendulum Swing Up Starting\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

	/* Initialize position and motion variables */
	max_encoder_position = 0;
	global_max_encoder_position = 0;
	peaked = 0;
	handled_peak = 0;
	swing_up_state = 0;
	swing_up_state_prev = 0;
	zero_crossed = 0;
	stage_count = 0;
	/* Select initial amplitude for rotor impulse */
	stage_amp = STAGE_0_AMP;

	/* Optional encoder state reporting */
	//sprintf(tmp_string,"Current Position %0.2f\r\n", (encoder_position - encoder_position_down)/angle_scale);
	//HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

	//sprintf(tmp_string,"Current Position Down %0.2f\r\n", encoder_position_down/angle_scale);
	//HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

	/* Initiate first swing */
	swing_up_direction = FORWARD;
	BSP_MotorControl_Move(0, swing_up_direction, 150);
	BSP_MotorControl_WaitWhileActive(0);


	/* Enter Swing Up Loop */
	while (1)
	{
		HAL_Delay_ReImplemented(2);
		ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		/* Optional Swing Up progress reporting */
		//sprintf(tmp_string,"Rotor Impulse Amplitude %i Max Angle (degrees) %0.3f\r\n", stage_amp, fabs((float)(global_max_encoder_position)/(ENCODER_READ_ANGLE_SCALE)));
		//HAL_UART_Transmit(&huart2, (uint8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

		/* Break if pendulum angle relative to vertical meets tolerance (for clockwise or counter clockwise approach */
		if (fabs(encoder_position_steps - encoder_position_down - (int) (180 * angle_scale)) < START_ANGLE * angle_scale){
			break;
		}
		if (fabs(encoder_position_steps - encoder_position_down + (int)(180 * angle_scale)) < START_ANGLE * angle_scale){
			encoder_position_down = encoder_position_down - 2*(int)(180 * angle_scale);
			break;
		}

		if (zero_crossed)
		{
			zero_crossed = 0;
			// Push it aka put some more kinetic energy into the pendulum
			if (swing_up_state == 0){
				BSP_MotorControl_Move(0, swing_up_direction, stage_amp);
				BSP_MotorControl_WaitWhileActive(0);
				stage_count++;

				if (prev_global_max_encoder_position != global_max_encoder_position && stage_count > 4){
				if (abs(global_max_encoder_position) < 600){
					stage_amp = STAGE_0_AMP;
				}
				if (abs(global_max_encoder_position) >= 600 && abs(global_max_encoder_position) < 1000){
					stage_amp = STAGE_1_AMP;
				}
				if (abs(global_max_encoder_position) >= 1000){
					stage_amp = STAGE_2_AMP;
				}
				}
				prev_global_max_encoder_position = global_max_encoder_position;
				global_max_encoder_position = 0;
				ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
			}
		}


		// We have a peak but did not handle it yet
		if (peaked && !handled_peak)
		{
			// Ensure we only enter this branch one per peak
			handled_peak = 1;
			// Reset maximum encoder value to reassess after crossing the bottom
			max_encoder_position = 0;
			// Switch motor direction
			swing_up_direction = swing_up_direction == FORWARD ? BACKWARD : FORWARD;
		}

}

void HAL_Delay_ReImplemented( int time_miliseconds){

	// Storing start time
	clock_t start_time = clock();

	// looping till required time is not achieved
	while (clock() < start_time + time_miliseconds);

}

void Set_Torque_Current_Value(){
	SendCommand (COMMAND_SET_TORGE_UP);
}

void SendCommand (int type ){

	switch( type ){
		case COMMAND_SET_TORGE_UP:
			break;

	}
}
