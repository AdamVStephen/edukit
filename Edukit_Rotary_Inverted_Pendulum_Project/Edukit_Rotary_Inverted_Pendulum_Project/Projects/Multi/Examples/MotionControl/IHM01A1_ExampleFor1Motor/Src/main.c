
/*
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 *
 *    Acknowledgments to the invaluable development, support and guidance by
 *    Marco De Fazio, Giorgio Mariano, Enrico Poli, and Davide Ghezzi
 *    of STMicroelectronics
 *
 *    Acknowledgements to the innovative development, support and guidance by
 *    Markus Dauberschmidt for development of the Pendulum Swing Up algorithm
 *    Please see https://github.com/OevreFlataeker/steval_edukit_swingup
 *
 *              Motor Control Curriculum Feedback Control System
 *
 * Includes:
 * 		Stepper Motor Control interface based on the IHM01A1 and Nucleo F401RE
 * 		Optical Encoder interface supported by the Nucleo F401RE
 * 		Primary PID controller for Pendulum Angle Control
 * 		Secondary PID controller for Rotor Angle Control
 * 		User interfaces for remote access to system configuration including
 * 				Stepper Motor speed profile, current limits, and others
 * 				Control system parameters
 *
 *
 * @author  William J. Kaiser (UCLA Electrical and Computer Engineering).
 *
 * Application based on development by STMicroelectronics as described below
 *
 * @version V2.0
 * @date    January 13, 2021
 *
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 * @author  IPC Rennes
 * @version V1.10.0
 * @date    March 16th, 2018
 * @brief   This example shows how to use 1 IHM01A1 expansion board
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/*
 * Integrated Rotary Inverted Pendulum System Configuration
 *
 * ********** Primary Components *************************
 * Processor:			Nucleo F401RE
 * Motor Interface: 	IHM01A1 Stepper Motor Controller
 * Motor Power Supply:	12V 2A Supply
 * Encoder:				LPD3806-600BM-G5-24C 600 Pulse Per Revolution Incremental Rotary Encoder
 * Stepper Motor:		NEMA-17 17HS19-2004S  Stepper Motor
 *
 *********** Stepper Lead Assignment *********************
 *
 * Lead Color	IHM01A1 Terminal
 * 	 Blue			-A
 * 	 Red			+A
 * 	 Green			-B
 * 	 Black			+B
 *
 * Stepper Lead Extension Cable (if present) replaces Blue with White
 *
 * Caution: Please note that motors have been receieved from the vendor showing
 * reversal of Motor White and Motor Red.  Check rotor operation after assembly
 * and in initial testing.
 *
 * ********** Encoder Lead Assignment *********************
 *
 * Lead Color	Nucleo F401RE Terminal
 * 	Red 			5V
 * 	Black 			GND
 * 	White 			GPIO Dir3
 * 	Green 			GPIO Dir2
 *
 * Note: Optical Encoder is LPD3806-600BM-G5-24C This encoder requires an open collector pull up resistor.
 * Note: GPIO_PULLUP is set in HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder) of
 * stm32f4xx_hal_msp.c Line 217
 *
 * Initial Motor Speed Profiles, Torque Current, and Overcurrent Thresholds set in l6474_target_config.h
 *
 *
 *
 */




/*
 * System Configuration Parameters
 *
 * ENABLE_SUSPENDED_PENDULUM_CONTROL is set to 0 for Inverted Pendulum and
 * 									 set to 1 for Suspended Pendulum
 * ENABLE_DUAL_PID is set to 1 to enable control action
 *
 * High Speed 2 millisecond Control Loop delay, 500 Hz Cycle System Configuration
 *
 * Motor Speed Profile Configurations are:
 *
 *						MAX_ACCEL: 3000; 	MAX_DECEL 3000
 * High Speed Mode:		MAX_SPEED: 1000; 	MIN_SPEED 500
 * Medium Speed Mode:	MAX_SPEED: 1000; 	MIN_SPEED 300
 * Low Speed Mode:		MAX_SPEED: 1000; 	MIN_SPEED 200
 * Suspended Mode: 		MAX_SPEED: 1000; 	MIN_SPEED 200
 *
 */



/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "edukit_system.h"
//#include "control_loop.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>


/*
 * Motor Interface Data Structure
 *
 * Note that this is not required by default since Motor Profile is included
 * from l6474_target_config.h
 *
 * Note: This application is based on usage of l6474_target_config.h header
 * file for initialization of configuration of L6474
 */

L6474_Init_t gL6474InitParams = {
		MAX_ACCEL,           	/// Acceleration rate in step/s2. Range: (0..+inf).
		MAX_DECEL,           	/// Deceleration rate in step/s2. Range: (0..+inf).
		MAX_SPEED,              /// Maximum speed in step/s. Range: (30..10000].
		MIN_SPEED,              /// Minimum speed in step/s. Range: [30..10000).
		MAX_TORQUE_CONFIG, 		/// Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
		OVERCURRENT_THRESHOLD, 	/// Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
		L6474_CONFIG_OC_SD_ENABLE, /// Overcurrent shutwdown (OC_SD field of CONFIG register).
		L6474_CONFIG_EN_TQREG_TVAL_USED, /// Torque regulation method (EN_TQREG field of CONFIG register).
		L6474_STEP_SEL_1_16, 	/// Step selection (STEP_SEL field of STEP_MODE register).
		L6474_SYNC_SEL_1_2, 	/// Sync selection (SYNC_SEL field of STEP_MODE register).
		L6474_FAST_STEP_12us, 	/// Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
		L6474_TOFF_FAST_8us, 	/// Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
		3,   					/// Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
		21, 					/// Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
		L6474_CONFIG_TOFF_044us, /// Target Swicthing Period (field TOFF of CONFIG register).
		L6474_CONFIG_SR_320V_us, /// Slew rate (POW_SR field of CONFIG register).
		L6474_CONFIG_INT_16MHZ, /// Clock setting (OSC_CLK_SEL field of CONFIG register).
		(L6474_ALARM_EN_OVERCURRENT | L6474_ALARM_EN_THERMAL_SHUTDOWN
				| L6474_ALARM_EN_THERMAL_WARNING | L6474_ALARM_EN_UNDERVOLTAGE
				| L6474_ALARM_EN_SW_TURN_ON | L6474_ALARM_EN_WRONG_NPERF_CMD) /// Alarm (ALARM_EN register).
};



 typedef enum {
  STATE_INITIALIZATION = 0,
  STATE_PENDULUM_STABLIZATION = 1,
  STATE_SWING_UP = 2,
  STATE_MAIN = 3
 } STATE;

 /*
  * Frame definition
  *
  * */
#pragma pack(push, 1)
 typedef struct
 {
 	int32_t positionRotor;
 	uint32_t encoder_counter;
 	uint32_t  Pwm1Counter;
// 	uint32_t CYCCNT;
// 	int32_t LOOP_BACK_rotor_control_target_steps;
// 	uint32_t LOOP_BACK_L6474_Board_Pwm1Period;
// 	uint8_t  LOOP_BACK_gpioState;
// 	uint8_t  LOOP_BACK_break_Control_Loop;
// 	uint8_t  LOOP_BACK_state;

 } DataFrameSend;
#pragma pack(pop)

#define DataFrameSend_SIZE  12

 #pragma pack(push, 1)
 typedef struct
 {
	int32_t control_target_steps;
	uint32_t Pwm1Period;
	uint8_t gpioState;
	uint8_t break_Control_Loop;
	uint8_t state;
 } tt, *DataFrameReceive;
#pragma pack(pop)

#define DataFrameReceive_SIZE  11

 /* CMSIS */
#define ARM_MATH_CM4

/*
 * Apply Swing Up algorithm developed by Markus Dauberschmidt
 */

#define swing_up 1

/*
 * Apply acceleration
 */

#define ACCEL_CONTROL_DATA 0		// Set to 1 for display of timing data
#define ACCEL_CONTROL 1 			// Set to 1 to enable acceleration control. Set to 0 to use position target control.
#define PWM_COUNT_SAFETY_MARGIN 2
#define MAXIMUM_ACCELERATION 131071
#define MAXIMUM_DECELERATION 131071
#define MAXIMUM_SPEED 131071

volatile uint16_t gLastError;
/* Private function prototypes -----------------------------------------------*/
void read_float(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, float *float_return);
void Error_Handler(uint16_t error);
void read_int(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, int * int_return);
void read_char(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, char * char_return);
void select_mode_1(void);
void set_default_configurations(void);
void Main_StepClockHandler();
void apply_acceleration(float * acc, float* target_velocity_prescaled, float t_sample);

#define delayUS_ASM(us) do {\
		asm volatile (	"MOV R0,%[loops]\n\t"\
				"1: \n\t"\
				"SUB R0, #1\n\t"\
				"CMP R0, #0\n\t"\
				"BNE 1b \n\t" : : [loops] "r" (16*us) : "memory"\
		);\
} while(0)

int Delay_Pulse(){
	return desired_pwm_period == UINT32_MAX;
}

/*
 * PWM pulse (step) interrupt
 */
void Main_StepClockHandler() {
	/*
	 *  Stepper motor acceleration, speed, direction and position control developed by Ryan Nemiroff
	 */

	uint32_t desired_pwm_period_local = desired_pwm_period;

	/*
	 * Add time reporting
	 */

	clock_int_time = DWT->CYCCNT;

	if (desired_pwm_period_local != 0) {
		L6474_Board_Pwm1SetPeriod(desired_pwm_period_local);
		current_pwm_period = desired_pwm_period_local;
	}
}

void apply_acceleration(float * acc, float * target_velocity_prescaled, float t_sample) {
	/*
	 *  Stepper motor acceleration, speed, direction and position control developed by Ryan Nemiroff
	 */

	uint32_t current_pwm_period_local = current_pwm_period;
	uint32_t desired_pwm_period_local = desired_pwm_period;

	/*
	 * Add time reporting
	 */

	apply_acc_start_time = DWT->CYCCNT;


	motorDir_t old_dir = *target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

	if (old_dir == FORWARD) {
		if (*acc > MAXIMUM_ACCELERATION) {
			*acc = MAXIMUM_ACCELERATION;
		} else if (*acc < -MAXIMUM_DECELERATION) {
			*acc = -MAXIMUM_DECELERATION;
		}
	} else {
		if (*acc < -MAXIMUM_ACCELERATION) {
			*acc = -MAXIMUM_ACCELERATION;
		} else if (*acc > MAXIMUM_DECELERATION) {
			*acc = MAXIMUM_DECELERATION;
		}	
	}

	*target_velocity_prescaled += L6474_Board_Pwm1PrescaleFreq(*acc) * t_sample;
	motorDir_t new_dir = *target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

	if (*target_velocity_prescaled > L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED)) {
		*target_velocity_prescaled = L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED);
	} else if (*target_velocity_prescaled < -L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED)) {
		*target_velocity_prescaled = -L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED);
	}

	float speed_prescaled;
	if (new_dir == FORWARD) {
		speed_prescaled = *target_velocity_prescaled;
	} else {
		speed_prescaled = *target_velocity_prescaled * -1;
		if (speed_prescaled == 0) speed_prescaled = 0; // convert negative 0 to positive 0
	}


	uint32_t effective_pwm_period = desired_pwm_period_local;

	float desired_pwm_period_float = roundf(RCC_SYS_CLOCK_FREQ / speed_prescaled);
	if (!(desired_pwm_period_float < 4294967296.0f)) {
		desired_pwm_period_local = UINT32_MAX;
	} else {
		desired_pwm_period_local = (uint32_t)(desired_pwm_period_float);
	}

	if (old_dir != new_dir) {
		L6474_Board_SetDirectionGpio(0, new_dir);
	}

	if (current_pwm_period_local != 0) {
		uint32_t pwm_count = L6474_Board_Pwm1GetCounter();
		uint32_t pwm_time_left = current_pwm_period_local - pwm_count;
		if (pwm_time_left > PWM_COUNT_SAFETY_MARGIN) {
			if (old_dir != new_dir) {
				// pwm_time_left = effective_pwm_period - pwm_time_left; // One method for assignment of PWM period during switching directions. This has the effect of additional discrete step noise.
				pwm_time_left = effective_pwm_period; // Second method for assignment of PWM period during switching directions. This shows reduced discrete step noise.
			}

			uint32_t new_pwm_time_left = ((uint64_t) pwm_time_left * desired_pwm_period_local) / effective_pwm_period;
			if (new_pwm_time_left != pwm_time_left) {
				if (new_pwm_time_left < PWM_COUNT_SAFETY_MARGIN) {
					new_pwm_time_left = PWM_COUNT_SAFETY_MARGIN;
				}
				current_pwm_period_local = pwm_count + new_pwm_time_left;
				if (current_pwm_period_local < pwm_count) {
					current_pwm_period_local = UINT32_MAX;
				}

				L6474_Board_Pwm1SetPeriod(current_pwm_period_local);
				current_pwm_period = current_pwm_period_local;
			}
		}
	} else {
		L6474_Board_Pwm1SetPeriod(desired_pwm_period_local);
		current_pwm_period = desired_pwm_period_local;
	}

	desired_pwm_period = desired_pwm_period_local;

}

__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds)
{
	uint32_t clk_cycle_start = DWT->CYCCNT;

	/* Go to number of cycles for system */
	microseconds *= (RCC_HCLK_FREQ / 1000000);

	/* Delay till end */
	while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

__STATIC_INLINE void DWT_Delay_until_cycle(volatile uint32_t cycle)
{
	while (DWT->CYCCNT < cycle);
}

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
T_Serial_Msg Msg;

uint8_t RxBuffer[UART_RX_BUFFER_SIZE];
uint16_t Extract_Msg(uint8_t *CircularBuff, uint16_t StartPos, uint16_t LastPos, uint16_t BufMaxLen, T_Serial_Msg *Msg);


int32_t LOOP_BACK_rotor_control_target_steps;
uint32_t LOOP_BACK_L6474_Board_Pwm1Period;
uint8_t  LOOP_BACK_gpioState;
uint8_t  LOOP_BACK_break_Control_Loop;
uint8_t  LOOP_BACK_state;
DataFrameReceive rx_frame;
DataFrameSend tx_frame;
uint8_t rx_frameBuffer[ DataFrameReceive_SIZE ];
const uint8_t SYNC_BYTES[4] = {0x00, 0xAC, 0xDD, 0xDD};

char msg_cmd[ DataFrameSend_SIZE+sizeof(SYNC_BYTES)  ]; //256
STATE prev_state;
STATE state;

/* Acceleration control system variables */
volatile uint32_t apply_acc_start_time;
volatile uint32_t clock_int_time;
volatile uint32_t clock_int_tick;

/// PWM period variables used by step interrupt
volatile uint32_t desired_pwm_period;
volatile uint32_t current_pwm_period;

float target_velocity_prescaled;
int32_t enable_speed_prescale;

/* System data reporting */
char tmp_string[256];
char msg[192];
char msg_pad[64];
char test_msg[128];

/*
 * Timer 3, UART Transmit, and UART DMA Receive declarations
 */

TIM_HandleTypeDef htim3;

/*
  * Timer 3, UART Transmit, and UART DMA Receive declarations
  */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Control system output signal */
float rotor_control_target_steps;
float rotor_control_target_steps_curr;
float rotor_control_target_steps_prev;

/* Control system variables */
int rotor_position_delta;
int initial_rotor_position;
int cycle_count;
int i, j, k, m;
int ret;

/* PID control system variables */
float windup, rotor_windup;
float *current_error_steps, *current_error_rotor_steps;
float *sample_period, *sample_period_rotor;

/* Loop timing measurement variables */
int cycle_period_start;
int cycle_period_sum;
int enable_cycle_delay_warning;

/* State Feedback variables */
int enable_state_feedback;
float integral_compensator_gain;
float feedforward_gain;
float current_error_rotor_integral;

/* Reference tracking command */
float reference_tracking_command;

/* Pendulum position and tracking command */

/* Rotor position and tracking command */
int rotor_position_steps;
float rotor_position_command_steps;
float rotor_position_command_steps_pf, rotor_position_command_steps_pf_prev;
float rotor_position_command_deg;
float rotor_position_steps_prev, rotor_position_filter_steps, rotor_position_filter_steps_prev;
float rotor_position_diff, rotor_position_diff_prev;
float rotor_position_diff_filter, rotor_position_diff_filter_prev;
int rotor_target_in_steps;
int initial_rotor_position;

/* Rotor Plant Design variables */
int select_rotor_plant_design, enable_rotor_plant_design, enable_rotor_plant_gain_design;
int rotor_control_target_steps_int;
float rotor_damping_coefficient, rotor_natural_frequency;
float rotor_plant_gain;
float rotor_control_target_steps_gain;
float rotor_control_target_steps_filter_2, rotor_control_target_steps_filter_prev_2;
float rotor_control_target_steps_prev_prev, rotor_control_target_steps_filter_prev_prev_2;
float c0, c1, c2, c3, c4, ao, Wn2;
float fo_r, Wo_r, IWon_r, iir_0_r, iir_1_r, iir_2_r;

/* Encoder position variables */
uint32_t cnt3;
int range_error;

/* Angle calibration variables */
float encoder_position_offset;
float encoder_position_offset_zero;
int enable_angle_cal;
int enable_angle_cal_resp;
int offset_end_state;
int offset_start_index;
int angle_index;
int angle_avg_index;
int angle_avg_span;
int offset_angle[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
float encoder_position_offset_avg[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
int angle_cal_end;
int angle_cal_complete;

/* Swing Up system variables */
int enable_swing_up;
int enable_swing_up_resp;
motorDir_t swing_up_direction;
int swing_up_state, swing_up_state_prev;
int stage_count;
int stage_amp;

/* Initial control state parameter storage */
float init_r_p_gain, init_r_i_gain, init_r_d_gain;
float init_p_p_gain, init_p_i_gain, init_p_d_gain;
int init_enable_state_feedback;
float init_integral_compensator_gain;
float init_feedforward_gain;
int init_enable_state_feedback;
int init_enable_disturbance_rejection_step;
int init_enable_sensitivity_fnc_step;
int init_enable_noise_rejection_step;
int init_enable_rotor_plant_design;
int init_enable_rotor_plant_gain_design;


/* Slope correction system variables */
int slope;
int slope_prev;
float encoder_angle_slope_corr_steps;

/* Adaptive control variables */
float adaptive_error, adaptive_threshold_low, adaptive_threshold_high;
float error_sum_prev, error_sum, error_sum_filter_prev, error_sum_filter;
int adaptive_entry_tick, adaptive_dwell_period;
int enable_adaptive_mode, adaptive_state, adaptive_state_change;
float rotor_position_command_steps_prev;

/* Rotor impulse variables */
int rotor_position_step_polarity;
int impulse_start_index;

/* User configuration variables */
int clear_input;
uint32_t enable_control_action;
int max_speed_read, min_speed_read;
int select_suspended_mode;
int motor_response_model;
int enable_rotor_actuator_test, enable_rotor_actuator_control;
int enable_encoder_test;
int enable_rotor_actuator_high_speed_test;
int enable_motor_actuator_characterization_mode;
int motor_state;
float torq_current_val;


/* Rotor chirp system variables */
int enable_rotor_chirp;
int chirp_cycle;
int chirp_dwell_cycle;
float chirp_time;
float rotor_chirp_start_freq;
float rotor_chirp_end_freq;
float rotor_chirp_period ;
float rotor_chirp_frequency;
float rotor_chirp_amplitude;
int rotor_chirp_step_period;

float pendulum_position_command_steps;

/* Modulates sine tracking signal system variables */
int enable_mod_sin_rotor_tracking;
int enable_rotor_position_step_response_cycle;
int disable_mod_sin_rotor_tracking;
int sine_drive_transition;
float mod_sin_amplitude;
float rotor_control_sin_amplitude;
float rotor_sine_drive, rotor_sine_drive_mod;
float rotor_mod_control;
float mod_sin_carrier_frequency;

/* Pendulum impulse system variables */
int enable_pendulum_position_impulse_response_cycle;

/* Rotor high speed test system variables */
int swing_cycles, rotor_test_speed_min, rotor_test_speed_max;
int rotor_test_acceleration_max, swing_deceleration_max;
int start_angle_a[20], end_angle_a[20], motion_dwell_a[20];
int abs_encoder_position_prior, abs_encoder_position_after, abs_encoder_position_max;
uint16_t current_speed;

/*Pendulum system ID variable */
int enable_pendulum_sysid_test;


/* Rotor comb drive system variables */
int enable_rotor_tracking_comb_signal;
float rotor_track_comb_signal_frequency;
float rotor_track_comb_command;
float rotor_track_comb_amplitude;

/* Sensitivity function system variables */
int enable_disturbance_rejection_step;
int enable_noise_rejection_step;
int enable_plant_rejection_step;
int enable_sensitivity_fnc_step;
float load_disturbance_sensitivity_scale;

/* Noise rejection sensitivity function low pass filter */

float noise_rej_signal_filter, noise_rej_signal;
float noise_rej_signal_prev, noise_rej_signal_filter_prev;

/*
 * Real time user input system variables
 */

char config_message[16];
int config_command;
int display_parameter;
int step_size;
float adjust_increment;
int mode_index;

/* Real time data reporting index */
int report_mode;
int speed_scale;
int speed_governor;

/*
 * User selection mode values
 */

int mode_quit;			// Initiate exit from control loop
int mode_interactive;	// Enable continued terminal interactive user session
int mode_index_prev, mode_index_command;
int mode_transition_tick;
int mode_transition_state;
int transition_to_adaptive_mode;


/* System timing variables */

uint32_t tick, tick_cycle_current, tick_cycle_previous, tick_cycle_start,
tick_read_cycle, tick_read_cycle_start,tick_wait_start,tick_wait;

volatile uint32_t current_cpu_cycle, prev_cpu_cycle, last_cpu_cycle, target_cpu_cycle, prev_target_cpu_cycle;
volatile int current_cpu_cycle_delay_relative_report;

uint32_t t_sample_cpu_cycles;
float Tsample, Tsample_rotor, test_time;
float angle_scale;
int enable_high_speed_sampling;

/* Reset state tracking */
int reset_state;

/* Motor configuration */
uint16_t min_speed, max_speed, max_accel, max_decel;

/* Serial interface variables */
uint32_t RxBuffer_ReadIdx;
uint32_t RxBuffer_WriteIdx;
uint32_t readBytes;

//int    LOOP_BACK_rotor_control_target_steps;
//uint8_t  LOOP_BACK_gpioState;
//uint32_t LOOP_BACK_L6474_Board_Pwm1Period;
//uint8_t  LOOP_BACK_break_Control_Loop;


void initialize(){
	/* Initialize reset state indicating that reset has occurred */

	reset_state = 1;

	/* Initialize and enable cycle counter */
	ITM->LAR = 0xC5ACCE55; 	// at address 0xE0001FB0
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // at address 0xE000EDFC, CoreDebug_DEMCR_TRCENA_Msk = 0x01000000
	DWT->CTRL |= 1; 		// at address 0xE0001000
	DWT->CYCCNT = 0; 		// at address 0xE0001004

	/* initialize Integrator Mode time variables */
	apply_acc_start_time = 0;
	clock_int_time = 0;
	clock_int_tick = 0;

	/* Initialize PWM period variables used by step interrupt */
	desired_pwm_period = 0;
	current_pwm_period = 0;
	target_velocity_prescaled = 0;


//	LOOP_BACK_rotor_control_target_steps=0;
//	LOOP_BACK_gpioState=0;
//	LOOP_BACK_L6474_Board_Pwm1Period=0;
//	LOOP_BACK_break_Control_Loop = 0;

	/* Initialize default start mode and reporting mode */
	mode_index = 1;
	report_mode = 1;

	/*Initialize serial read variables */
	RxBuffer_ReadIdx = 0;
	RxBuffer_WriteIdx = 0;
	readBytes = 0;


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

	/* STM32xx HAL library initialization */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Default select_suspended_mode */
	select_suspended_mode = ENABLE_SUSPENDED_PENDULUM_CONTROL;

	//----- Initialize Motor Control Library
	/* Set the L6474 library to use 1 device */
	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);

	/* When BSP_MotorControl_Init is called with NULL pointer,                  */
	/* the L6474 registers and parameters are set with the predefined values from file   */
	/* l6474_target_config.h, otherwise the registers are set using the   */
	/* L6474_Init_t pointer structure                */
	/* The first call to BSP_MotorControl_Init initializes the first device     */
	/* whose Id is 0.                                                           */
	/* The nth call to BSP_MotorControl_Init initializes the nth device         */
	/* whose Id is n-1.                                                         */
	/* Uncomment the call to BSP_MotorControl_Init below to initialize the      */
	/* device with the structure gL6474InitParams declared in the the main.c file */
	/* and comment the subsequent call having the NULL pointer                   */
	//BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, NULL);

	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &gL6474InitParams);

	/* Initialize Timer and UART */
	MX_TIM3_Init();

	/* Initialize Pendulum Angle Encoder offset */
	HAL_Delay(10);
	/* Initialize UART communication port */
	MX_USART2_UART_Init();

	/* Motor Range Initialization */
	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, MAX_SPEED_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, MIN_SPEED_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, MAX_SPEED_LOWER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, MIN_SPEED_LOWER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetAcceleration(0, MAX_ACCEL_UPPER_INIT);
	HAL_Delay(1);
	BSP_MotorControl_SetDeceleration(0, MAX_DECEL_UPPER_INIT);
	HAL_Delay(1);

	/* Default Starting Control Configuration */
	max_accel = MAX_ACCEL;
	max_decel = MAX_DECEL;
	max_speed = MAX_SPEED_MODE_1;
	min_speed = MIN_SPEED_MODE_1;
	HAL_Delay(1);
	BSP_MotorControl_SetMaxSpeed(0, max_speed);
	HAL_Delay(1);
	BSP_MotorControl_SetMinSpeed(0, min_speed);
	HAL_Delay(1);
	BSP_MotorControl_SetAcceleration(0, max_accel);
	HAL_Delay(1);
	BSP_MotorControl_SetDeceleration(0, max_decel);
	HAL_Delay(1);

	/* Default torque current */
	torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);


	/* Enable State Feedback mode and Integral Action Compensator by default and set
	 * precompensation factor to unity
	 */
	enable_state_feedback = 1;
	integral_compensator_gain = 0;
	feedforward_gain = 1;

	/* Disable adaptive_mode by default */
	enable_adaptive_mode = 0;

	/* DMA Buffer declarations */
	/* Start DMA just once because it's configured in "circular" mode */
	HAL_UART_Receive_DMA(&huart2, RxBuffer, UART_RX_BUFFER_SIZE);

	/* Motor Interface and Encoder initialization */
	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

	/* Attach the function Error_Handler (defined below) to the error Handler*/
	BSP_MotorControl_AttachErrorHandler(Error_Handler);

	/* Encoder initialization */
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

//
//	/* Configure controller filter and sample time parameters */
//	*deriv_lp_corner_f = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
//	*deriv_lp_corner_f_rotor = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	t_sample_cpu_cycles = (uint32_t) round(T_SAMPLE_DEFAULT * RCC_HCLK_FREQ);
	Tsample = (float) t_sample_cpu_cycles / RCC_HCLK_FREQ;
	*sample_period = Tsample;
	Tsample_rotor = Tsample;
	*sample_period_rotor = Tsample_rotor;


	enable_adaptive_mode = ENABLE_ADAPTIVE_MODE;
	adaptive_threshold_low = ADAPTIVE_THRESHOLD_LOW;
	adaptive_threshold_high = ADAPTIVE_THRESHOLD_HIGH;
	adaptive_state = ADAPTIVE_STATE;
	adaptive_state_change = 0;
	adaptive_dwell_period = ADAPTIVE_DWELL_PERIOD;

}
void initialize_main_loop(){

	prev_state = STATE_INITIALIZATION;
	state  = STATE_INITIALIZATION;

	LOOP_BACK_rotor_control_target_steps = 0;
	LOOP_BACK_L6474_Board_Pwm1Period = 0;
	LOOP_BACK_gpioState = 0;
	LOOP_BACK_break_Control_Loop = 0;
	LOOP_BACK_state = 0;

	mode_interactive = 0;

	/* Start timer for configuration command read loop */
	tick_read_cycle_start = HAL_GetTick();
	/* Configuration command read loop */
	set_default_configurations();

	/* Set Motor Speed Profile and torque current */
	BSP_MotorControl_SoftStop(0);
	BSP_MotorControl_WaitWhileActive(0);
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
	BSP_MotorControl_SetMaxSpeed(0, max_speed);
	BSP_MotorControl_SetMinSpeed(0, min_speed);
	BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
	BSP_MotorControl_SetDeceleration(0, MAX_DECEL);


	/*
	 * *************************************************************************************************
	 *
	 * Control System Initialization Sequence
	 *
	 * *************************************************************************************************
	 */

	/* Setting enable_control_action enables control loop */
	enable_control_action = ENABLE_CONTROL_ACTION;

	/*
	 * Set Motor Position Zero occuring only once after reset and suppressed thereafter
	 * to maintain angle calibration
	 */

	if (reset_state == 1){
		rotor_position_set();
	}

	//######## Jawad ************** Modification ########################################

	ret = rotor_position_read(&rotor_position_steps);
	/*
	 * Determination of vertical down orientation of the pendulum is required
	 * to establish the reference angle for the vertical upward control setpoint.
	 *
	 * This is measured when the pendulum is determined to be motionless.
	 *
	 * The user is informed to allow the pendulum to remain at rest.
	 *
	 * Motion is detected in the loop below.  Exit from the loop and
	 * initiation of control occurs next.
	 *
	 * Prior to measurement, and due to previous action, the Pendulum may be poised
	 * at upright orientation.
	 *
	 * A small stimulus is applied to ensure Pendulum will fall to Suspended orientation
	 * in the event that it may be finely balanced in the vertical position
	 *
	 */
	BSP_MotorControl_GoTo(0, 3);
	BSP_MotorControl_WaitWhileActive(0);
	HAL_Delay(150);
	BSP_MotorControl_GoTo(0, -3);
	BSP_MotorControl_WaitWhileActive(0);
	HAL_Delay(150);
	BSP_MotorControl_GoTo(0, 3);
	BSP_MotorControl_WaitWhileActive(0);
	HAL_Delay(150);
	BSP_MotorControl_GoTo(0, 0);
	BSP_MotorControl_WaitWhileActive(0);

}

int main(void) {


	/*
		 * Primary Controller Mode Configuration Loop
		 *
		 * Outer Loop acquires configuration command
		 * Inner Loop includes control system
		 * Inner Loop exits to Outer Loop upon command or exceedance
		 * of rotor or pendulum angles
		 *
		 * Outer Loop provided user-selected system reset option during
		 * data entry by Serial Interface
		 *
		 */


	initialize();

	while (1) {

		initialize_main_loop();

		/*
		 * *************************************************************************************************
		 *
		 * Control Loop Start
		 *
		 * *************************************************************************************************
		 */
		Send_Output_Response();
		while (enable_control_action == 1) {

			//Check if a break loop command was received. if Yes, break
			if( Process_Input_Requests() == 0)
				break;

			Send_Output_Response( );
		}
		/*
		 * *************************************************************************************************
		 *
		 * Control Loop Exit
		 *
		 * *************************************************************************************************
		 */

		/*
		 * Control System Exit Loop
		 */
		if (ACCEL_CONTROL == 1) {
			desired_pwm_period = 0;
			current_pwm_period = 0;
		}

		/*
		 * Restore rotor position at low speed profile
		 */
		ret = rotor_position_read(&rotor_position_steps);
		BSP_MotorControl_GoTo(0, 0);
		BSP_MotorControl_SoftStop(0);

		/*
		 * Terminate motor control
		 */

		ret = rotor_position_read(&rotor_position_steps);

		/*
		 * System software reset
		 */

		NVIC_SystemReset();

	}
}


/* TIM3 init function */
void MX_TIM3_Init(void) {

	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler(0);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler(0);
	}

}

/* USART2 init function */

void MX_USART2_UART_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
							;

	huart2.Instance = USART2;
	huart2.Init.BaudRate = SAMPLE_BAUD_RATE;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler(0);
	}

	/* USART2 RX DMA Init */
	hdma_usart2_rx.Instance = DMA1_Stream5;
	hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

	if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
		Error_Handler(0);
	}
	__HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);
}

/**
 * @brief  This function is the User handler for the flag interrupt
 * @param  None
 * @retval None
 */
void MyFlagInterruptHandler(void) {
	/* Get the value of the status register via the L6474 command GET_STATUS */
	uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);

	/* Check HIZ flag: if set, power brigdes are disabled */
	if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ) {
		// HIZ state
		// Action to be customized
	}

	/* Check direction bit */
	if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR) {
		// Forward direction is set
		// Action to be customized
	} else {
		// Backward direction is set
		// Action to be customized
	}

	/* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
	/* This often occures when a command is sent to the L6474 */
	/* while it is in HIZ state */
	if ((statusRegister & L6474_STATUS_NOTPERF_CMD)
			== L6474_STATUS_NOTPERF_CMD) {
		// Command received by SPI can't be performed
		// Action to be customized
	}

	/* Check WRONG_CMD flag: if set, the command does not exist */
	if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD) {
		//command received by SPI does not exist
		// Action to be customized
	}

	/* Check UVLO flag: if not set, there is an undervoltage lock-out */
	if ((statusRegister & L6474_STATUS_UVLO) == 0) {
		//undervoltage lock-out
		// Action to be customized
	}

	/* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_WRN) == 0) {
		//thermal warning threshold is reached
		// Action to be customized
	}

	/* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_SD) == 0) {
		//thermal shut down threshold is reached
		// Action to be customized
	}

	/* Check OCD  flag: if not set, there is an overcurrent detection */
	if ((statusRegister & L6474_STATUS_OCD) == 0) {
		//overcurrent detection
		// Action to be customized
	}

}


/**
 * @brief  This function is executed in event of error occurrence.
 * @param  error number of the error event
 * @retval None
 */
void Error_Handler(uint16_t error) {
	/* Backup error number */
	gLastError = error;

	/* Infinite loop */
	while (1) {
	}
}


/*
 ******************************************************************************
 *
 * Edukit System Functions Definitions
 *
 ******************************************************************************
 */

/*
 * Returns true if the two arguments have opposite sign, false if not
 * @retval bool.
 * Developed and provided by Markus Dauberschmidt
 */

__INLINE bool oppositeSigns(int x, int y) {
	return ((x ^ y) < 0);
}


/*
 * Rotor position set
 */

void rotor_position_set(void) {
	uint32_t rotor_position_u;
	rotor_position_u = BSP_MotorControl_GetPosition(0);
	BSP_MotorControl_SetHome(0, rotor_position_u);
}

/*
 * Rotor position read (returns signed integer)
 *
 * Returns error if overflow detected
 *
 */

__INLINE int rotor_position_read(int *rotor_position) {
	uint32_t rotor_position_u;
	int range_error;
	rotor_position_u = BSP_MotorControl_GetPosition(0);

	if (rotor_position_u > 2147483648) {
		*rotor_position = (int) (rotor_position_u) - 4294967296;
	} else {
		*rotor_position = (int) (rotor_position_u);
	}
	range_error = 0;
	if (*rotor_position <= -2147483648) {
		range_error = -1;
		*rotor_position = -2147483648;
	}
	if (*rotor_position >= 2147483647) {
		range_error = 1;
		*rotor_position = 2147483647;
	}
	return range_error;
}


/*
 * Configure system based on user selection
 */

void set_default_configurations(void){

	enable_rotor_tracking_comb_signal = 0;
	rotor_track_comb_amplitude = 0;
	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;

//############## Jawad Modification  -->> ###############################
	enable_state_feedback = 0;
	select_suspended_mode = 0;

	enable_rotor_plant_design = 0;
	enable_rotor_plant_gain_design = 0;
	enable_rotor_position_step_response_cycle = 0;
	enable_pendulum_position_impulse_response_cycle = 0;
	enable_rotor_chirp = 0;
	enable_mod_sin_rotor_tracking = 1;
	enable_angle_cal = 1;
	enable_swing_up = 1;
	L6474_SetAnalogValue(0, L6474_TVAL, TORQ_CURRENT_DEFAULT);
	return;
//#######################################################################

}


/*
 *Jawad Modification ======== ########################
 *Send_Output_Response

 */


/*
 *Jawad Modification ======== ########################
 * Read input frame and decode it
 */
void Send_Output_Response(  ){
	ret = rotor_position_read(&rotor_position_steps);

//DataFrameSend_SIZE,DataFrameReceive_SIZE


//############### validation (Uncomment lines 6296 to 6299)##############################
//	tx_frame.positionRotor = 9; //rotor_position_steps;
//	tx_frame.encoder_counter = 8;// __HAL_TIM_GET_COUNTER( (TIM_HandleTypeDef*) &htim3);
//	tx_frame.Pwm1Counter = 7;//L6474_Board_Pwm1GetCounter();
//	tx_frame.CYCCNT = 6;//DWT->CYCCNT;
//	tx_frame.LOOP_BACK_rotor_control_target_steps = 5;
//	tx_frame.LOOP_BACK_gpioState = 2;
//	tx_frame.LOOP_BACK_L6474_Board_Pwm1Period = 1;
//	tx_frame.LOOP_BACK_break_Control_Loop = 0;
//	tx_frame.LOOP_BACK_state = 4;

//#################VALIDATION ###########################

	tx_frame.positionRotor = rotor_position_steps;
	tx_frame.encoder_counter =  __HAL_TIM_GET_COUNTER( (TIM_HandleTypeDef*) &htim3);
	tx_frame.Pwm1Counter = L6474_Board_Pwm1GetCounter();
//	tx_frame.CYCCNT = DWT->CYCCNT;

//	tx_frame.LOOP_BACK_rotor_control_target_steps = LOOP_BACK_rotor_control_target_steps;
//	tx_frame.LOOP_BACK_gpioState = LOOP_BACK_gpioState;
//	tx_frame.LOOP_BACK_L6474_Board_Pwm1Period = LOOP_BACK_L6474_Board_Pwm1Period;
//	tx_frame.LOOP_BACK_break_Control_Loop = LOOP_BACK_break_Control_Loop;
//	tx_frame.LOOP_BACK_state = LOOP_BACK_state;

	memcpy(msg_cmd, &SYNC_BYTES, sizeof(SYNC_BYTES) );
	memcpy(msg_cmd+sizeof(SYNC_BYTES), &tx_frame, DataFrameSend_SIZE );

	ret = HAL_UART_Transmit(&huart2, (uint8_t*) msg_cmd, DataFrameSend_SIZE+sizeof(SYNC_BYTES), HAL_MAX_DELAY);

}


void STATE_Pendulum_Stablisation_Postprocessing(){
	HAL_Delay(100);

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
		current_cpu_cycle = 0;
		speed_scale = DATA_REPORT_SPEED_SCALE;
		speed_governor = 0;
		encoder_position_offset = 0;
		encoder_position_offset_zero = 0;

		for (m = 0; m < ANGLE_CAL_OFFSET_STEP_COUNT + 1; m++){
			offset_angle[m] = 0;
		}

		/* Initialize UART receive system */
		__HAL_DMA_RESET_HANDLE_STATE(&hdma_usart2_rx);

		/*
		 * Record user selected operation variable values.  Values will be
		 * restored after Swing Up completion or after Angle Calibration
		 * completion
		 */

		/* Initial control state parameter storage */
		init_enable_state_feedback = enable_state_feedback;
		init_integral_compensator_gain = integral_compensator_gain;
		init_feedforward_gain = feedforward_gain;
		init_enable_state_feedback = enable_state_feedback;
		init_enable_disturbance_rejection_step = enable_disturbance_rejection_step;
		init_enable_sensitivity_fnc_step = enable_sensitivity_fnc_step;
		init_enable_noise_rejection_step = enable_noise_rejection_step;
		init_enable_rotor_plant_design = enable_rotor_plant_design;
		init_enable_rotor_plant_gain_design = enable_rotor_plant_gain_design;

		if(select_suspended_mode == 1){
			load_disturbance_sensitivity_scale = 1.0;
		}
		if(select_suspended_mode == 0){
			load_disturbance_sensitivity_scale = LOAD_DISTURBANCE_SENSITIVITY_SCALE;
		}


		/*
		 * Initiate Pendulum Swing Up with automatic system requiring no user action
		 *
		 * This system was developed by Markus Dauberschmidt see
		 * https://github.com/OevreFlataeker/steval_edukit_swingup
		 *
		 */


		if (enable_swing_up == 1 && select_suspended_mode == 0){

			/*
			 * Apply controller parameters for initial operation at completion of
			 * Swing Up
			 */
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
			torq_current_val = MAX_TORQUE_SWING_UP;
			L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);


			//******Pendulum Swing Up Starting

			/* Initialize position and motion variables */
			/* Select initial amplitude for rotor impulse */
			stage_amp = STAGE_0_AMP;


			/* Initiate first swing */
			swing_up_direction = FORWARD;
			BSP_MotorControl_Move(0, swing_up_direction, 150);
			BSP_MotorControl_WaitWhileActive(0);

		}

}
void STATE_Swinp_Up_Postprocessing(){

	enable_control_action = 1;

	if (ACCEL_CONTROL == 1) {
		BSP_MotorControl_HardStop(0);
		L6474_CmdEnable(0);
		target_velocity_prescaled = 0;
		L6474_Board_SetDirectionGpio(0, BACKWARD);
	}

	/*
	 * Set Torque Current to value for normal operation
	 */
	torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);

	target_cpu_cycle = DWT->CYCCNT;
	prev_cpu_cycle = DWT->CYCCNT;

	//HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
}

/*
 *Jawad Modification ======== ########################
 * Read input frame and decode it
 */
int read_Frame() {

	int k;

	/* Number of bytes to be analyzed */
	uint16_t NumNewByte = 0;

	RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
	uint16_t LastPos  = RxBuffer_WriteIdx;
	uint16_t StartPos = RxBuffer_ReadIdx;

	/* Two index for ByteStuffing process  */
	uint16_t BuffIdx;
	/* Circular buffer index */
	uint16_t MsgIdx;
	if (LastPos >= StartPos) {
		NumNewByte = LastPos - StartPos;
	} else {
		NumNewByte = UART_RX_BUFFER_SIZE + LastPos - StartPos;
	}

	if( NumNewByte < DataFrameReceive_SIZE ) return 0;

	NumNewByte = DataFrameReceive_SIZE;

	BuffIdx = StartPos;

	//############ Validation Begin################################################################
//	                           |  int32                |     uint32           |uint8|uint8| uint8
//	uint8_t buff[ UART_RX_BUFFER_SIZE] =
//							  {0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//			                   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05,  //1, 3, 2, 4, 5
//							   0x01, 0x00, 0x00, 0x00,  0x03, 0x00, 0x00, 0x00, 0x02, 0x04, 0x05   //1, 3, 2, 4, 5
//	};
	//############Validation End #########################################################################

	for (MsgIdx = 0; MsgIdx < NumNewByte; MsgIdx++) {
		rx_frameBuffer[MsgIdx] = RxBuffer[BuffIdx];
		BuffIdx++;
		if (BuffIdx >= UART_RX_BUFFER_SIZE) {
			BuffIdx = 0;
		}
	}
	//memcpy(rx_frameBuffer, RxBuffer+StartPos, NumNewByte );
	rx_frame = (DataFrameReceive)rx_frameBuffer;

	RxBuffer_ReadIdx = (RxBuffer_ReadIdx + NumNewByte) % UART_RX_BUFFER_SIZE;

	return 1;



}

/*
 *Jawad Modification ======== ########################
 *Command processor
 */
int Process_Input_Requests(  ){


	if ( read_Frame() ) // Message found
	{

//		LOOP_BACK_rotor_control_target_steps = rx_frame->control_target_steps;
//		LOOP_BACK_L6474_Board_Pwm1Period = rx_frame->Pwm1Period;
//		LOOP_BACK_gpioState = rx_frame->gpioState;
//		LOOP_BACK_break_Control_Loop = rx_frame->break_Control_Loop;
//		LOOP_BACK_state = rx_frame->state;


		prev_state = state;
		state = rx_frame->state;

		if( prev_state==STATE_PENDULUM_STABLIZATION &&  state == STATE_SWING_UP ){
			STATE_Pendulum_Stablisation_Postprocessing();
		}
		if( prev_state==STATE_SWING_UP &&  state == STATE_MAIN){ //transitioning to Main from switng-up
			STATE_Swinp_Up_Postprocessing();
		}

		if( state == STATE_PENDULUM_STABLIZATION){//do nothing just delay and wait
			HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
		}else if( state == STATE_SWING_UP){//swing up state

			swing_up_direction = rx_frame->gpioState;
			stage_amp = rx_frame->control_target_steps;
			if( stage_amp > 0){
				BSP_MotorControl_Move(0, swing_up_direction, stage_amp);
				BSP_MotorControl_WaitWhileActive(0);
			}
		}else{
			if( rx_frame->gpioState != UNKNOW_DIR )
				L6474_Board_SetDirectionGpio(0, rx_frame->gpioState);

			if( rx_frame->Pwm1Period != 0 )
				L6474_Board_Pwm1SetPeriod(rx_frame->Pwm1Period);
			//else
			//	BSP_MotorControl_GoTo(0,rx_frame->control_target_steps );
		}

		if( rx_frame->break_Control_Loop == 1){
			return 0;
		}



	}
	return 1;
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif



/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/






