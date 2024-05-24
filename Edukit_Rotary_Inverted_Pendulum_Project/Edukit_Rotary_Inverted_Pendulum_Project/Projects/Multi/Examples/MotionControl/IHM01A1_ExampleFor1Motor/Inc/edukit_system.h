




/*
 ******************************************************************************
 * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c
 *
 *    Acknowledgments to the invaluable development, support and guidance by
 *    Marco De Fazio, Giorgio Mariano, Enrico Poli, and Davide Ghezzi
 *    of STMicroelectronics
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
 * @author  William J. Kaiser (UCLA Electrical and Computer Engineering).
 *
 * Application based on development by STMicroelectronics as described below
 *
 * @version V1.0
 * @date    May 15th, 2019
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
 * Control System and Motor Configuration Parameter Definitions
 */

/*
 * Sample rates defined for controller operation
 *
 * Note that these values scale derivative and integral computations
 *
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EDUKIT_H
#define __EDUKIT_H

#define RCC_SYS_CLOCK_FREQ 84000000 // should equal HAL_RCC_GetSysClockFreq()
#define RCC_HCLK_FREQ 84000000 // should equal HAL_RCC_GetHCLKFreq()


#define T_SAMPLE_DEFAULT 0.002

#define ENABLE_HIGH_SPEED_SAMPLING_MODE 			0
#define SAMPLE_BAUD_RATE							230400

#define CONTROLLER_GAIN_SCALE 						1
#define STEPPER_READ_POSITION_STEPS_PER_DEGREE 		8.888889	//	Stepper position read value in steps per degree
#define STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE 	STEPPER_READ_POSITION_STEPS_PER_DEGREE
#define ENCODER_READ_ANGLE_SCALE 					6.666667 // Angle Scale 6.66667 for 600 Pulse Per Rev Resolution Optical Encoder
#define FULL_STATE_FEEDBACK_SCALE 					1.00 // Scale factor for Full State Feedback Architecture

#define ENABLE_CYCLE_DELAY_WARNING 1			// Enable warning and control loop exit if loop delay exceeds threshold

#define ENCODER_ANGLE_POLARITY -1.0				// Note that physical system applies negative polarity to pendulum angle
												// by definition of coordinate system.

#define CYCLE_LIMIT 100000 						// Cycle limit determines run time as product of cycle limit and cycle time (typical 5 msec).
#define ENABLE_CYCLE_INFINITE 1 				// ENABLE_CYCLE_INFINITE set to 1 if continuous operation is to be enabled


#define MAX_TORQUE_CONFIG 800 					// 400 Selected Value for normal control operation
#define MAX_TORQUE_SWING_UP 800					// 800 Selected Value for Swing Up operation

#define OVERCURRENT_THRESHOLD 2000				// 2000 Selected Value for Integrated Rotary Inverted Pendulum System
#define SHUTDOWN_TORQUE_CURRENT 0				// 0 Selected Value for Integrated Rotary Inverted Pendulum System
#define TORQ_CURRENT_DEFAULT MAX_TORQUE_CONFIG				// Default torque current	// Default torque current

/*
 * Note that Speed Profiles are set at run time during execution.
 *
 * Default speed Profiles are set in l6474_target_config.h
 */

#define MAX_SPEED_UPPER_INIT 10000						// Initialization value
#define MIN_SPEED_UPPER_INIT 10000						// Initialization value
#define MAX_SPEED_LOWER_INIT 30							// Initialization value
#define MIN_SPEED_LOWER_INIT 30							// Initialization value
#define MAX_ACCEL_UPPER_INIT 10000			 			// Initialization value
#define MAX_DECEL_UPPER_INIT 10000	 					// Initialization value

/*
 * Configuration of Motor Speed Profile at initialization
 */

#define MAX_SPEED 2000
#define MIN_SPEED 800
#define MAX_ACCEL 6000
#define MAX_DECEL 6000

#define MAX_SPEED_MODE_2 2000
#define MIN_SPEED_MODE_2 1000
#define MAX_SPEED_MODE_1 2000
#define MIN_SPEED_MODE_1 800
#define MAX_SPEED_MODE_3 2000
#define MIN_SPEED_MODE_3 600
#define MAX_SPEED_MODE_4 2000
#define MIN_SPEED_MODE_4 400
#define MAX_SPEED_MODE_5 2000
#define MIN_SPEED_MODE_5 800

#define ENABLE_SUSPENDED_PENDULUM_CONTROL 0     // Set to 0 for Inverted Pendulum Mode - Set to 1 for Suspended Pendulum Mode

#define ENCODER_START_OFFSET 0 				// Encoder configurations may display up to 1 degree initial offset
#define ENCODER_START_OFFSET_DELAY 0			// Encoder offset delay limits application of initial offset
#define START_ANGLE 1							// Pendulum angle tolerance for system pendulum orientation at start
#define START_ANGLE_DELAY 0						// Delay at start for orientation of pendulum upright

/*
 * Pendulum Swing Up Configuration
 */

/*
 * Iinital Measurement of Edukit Platform angle relative to vertical.
 *
 * The Edukit system may be resting on a sloped surface.  Therefore, accurate measurement of Pendulum upright angle
 * includes a slope error.  This is corrected for by the Angle Calibration System that operates at start time
 *
 */

#define ENABLE_ANGLE_CAL 1
#define ANGLE_CAL_OFFSET_STEP_COUNT 1801	// Full span angle range from negative to positive rotor angle
#define ANGLE_AVG_SPAN 50 					// Angle element smoothing span in rotor steps reduces full span by twice set value
#define ANGLE_CAL_ZERO_OFFSET_DWELL 5000	// 10 second zero offset measurement period
#define ANGLE_CAL_ZERO_OFFSET_SETTLING 2000 // 4 second settling period after offset correction
#define ANGLE_CAL_COMPLETION 2000			// 4 seconds settling period after final offset update prior to assigning new controller


/*
 * Single PID, Dual PID and LQR Controllers are implemented as summation of Primary
 * and Secondary PID controller structures.  These two controller outputs are summed
 * and supplied to Rotor Control
 *
 * PID Controller parameters.  LQR Controllers are implemented with integral gain of 0.
 *
 */

#define ENABLE_PID_INTEGRATOR_LIMIT 0			// Enable PID filter integrator limit
#define PRIMARY_WINDUP_LIMIT 1000				// Integrator wind up limits for PID Pendulum controller
#define SECONDARY_WINDUP_LIMIT 1000				// Integrator wind up limits for PID Rotor controller

/*
 * High Speed Mode Values: 5, 25, 30
 */
#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY 10  		// 10 - Corner frequency of low pass filter of Primary PID derivative
#define LP_CORNER_FREQ_ROTOR 100 						// 100 - Corner frequency of low pass filter of Rotor Angle
#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR 50 	// 50 - Corner frequency of low pass filter of Secondary PID derivative
#define LP_CORNER_FREQ_STEP 50							// Low pass filter operating on rotor reference step command signal


/* Mode 1 is Dual PID Demonstration Mode */

#define PRIMARY_PROPORTIONAL_MODE_1 	300
#define PRIMARY_INTEGRAL_MODE_1     	0.0
#define PRIMARY_DERIVATIVE_MODE_1   	30

#define SECONDARY_PROPORTIONAL_MODE_1 	15.0
#define SECONDARY_INTEGRAL_MODE_1     	0.0
#define SECONDARY_DERIVATIVE_MODE_1   	7.5

#define PRIMARY_PROPORTIONAL_MODE_2 	518.0
#define PRIMARY_INTEGRAL_MODE_2     	0
#define PRIMARY_DERIVATIVE_MODE_2   	57.0

#define SECONDARY_PROPORTIONAL_MODE_2 	2.20
#define SECONDARY_INTEGRAL_MODE_2     	0.0
#define SECONDARY_DERIVATIVE_MODE_2   	4.82

#define PRIMARY_PROPORTIONAL_MODE_3 	300
#define PRIMARY_INTEGRAL_MODE_3     	0.0
#define PRIMARY_DERIVATIVE_MODE_3   	30.0

#define SECONDARY_PROPORTIONAL_MODE_3 	15.0
#define SECONDARY_INTEGRAL_MODE_3     	0.0
#define SECONDARY_DERIVATIVE_MODE_3   	15.0

#define PRIMARY_PROPORTIONAL_MODE_5 	300
#define PRIMARY_INTEGRAL_MODE_5     	0.0
#define PRIMARY_DERIVATIVE_MODE_5   	30.0

#define SECONDARY_PROPORTIONAL_MODE_5 	15.0
#define SECONDARY_INTEGRAL_MODE_5     	0.0
#define SECONDARY_DERIVATIVE_MODE_5   	15.0

/*
 * Single PID Mode Gains
 */

#define ROTOR_PID_PROPORTIONAL_GAIN_SINGLE_PID_MODE  15.0
#define ROTOR_PID_INTEGRAL_GAIN_SINGLE_PID_MODE		 0.0
#define ROTOR_PID_DIFFERENTIAL_GAIN_SINGLE_PID_MODE	 7.5

/*
 * Mode 4 is Dual PID Suspended Demonstration Mode
 */

#define PRIMARY_PROPORTIONAL_MODE_4 	-10.0
#define PRIMARY_INTEGRAL_MODE_4     	 0.0
#define PRIMARY_DERIVATIVE_MODE_4   	-5.0

#define SECONDARY_PROPORTIONAL_MODE_4 	-2.0
#define SECONDARY_INTEGRAL_MODE_4     	 0.0
#define SECONDARY_DERIVATIVE_MODE_4   	-2.0


#define DEFAULT_START_MODE	mode_1

#define ENABLE_ADAPTIVE_MODE 0
#define ADAPTIVE_THRESHOLD_LOW 30				// Default value 30
#define ADAPTIVE_THRESHOLD_HIGH 2				// Default value 2
#define ADAPTIVE_STATE 0
#define ADAPTIVE_DWELL_PERIOD 2000				// Determines dwell period during state transition

#define USER_TRANSITION_DWELL 500

/*
 * START_DEFAULT_MODE_TIME determines time delay for waiting for user input after start or reset.
 * For a time (in ticks) greater than this period, control will initiate with default mode 1 if
 * no user input appears.
 *
 * This permits system operation in default mode independent of external command from separate
 * host.
 */

#define START_DEFAULT_MODE_TIME 5000			// 5 second delay to permit user input after system start
												// If no user response then set default values
#define PENDULUM_ORIENTATION_START_DELAY 10000	// Time permitted to user to orient Pendulum vertical at start

#define INITIAL_START_DELAY 1000				// Determines time available for ensuring pendulum down and prior to user prompt
#define CONTROL_START_DELAY 1000 				// Determines time available to user after prompt for adjusting pendulum upright
#define INITIAL_PENDULUM_MOTION_TEST_DELAY 2000 // Determines delay time between successive evaluations of pendulum motion

#define ENABLE_CONTROL_ACTION 1							// Enable control operation - default value of 1 (may be disabled for test operations)
#define ENABLE_DUAL_PID 1						// Note ENABLE_DUAL_PID is set to 1 by default for summation of PID controllers
												// for either Dual PID or LQR systems

#define STATE_FEEDBACK_CONFIG_ENABLE 1			// Default selection of Dual PID Architecture - Set to 1 for State Feedback			1

/*
 * Swing Up System Parameters : Swing Up Algorithm developed and provided by Markus Dauberschmidt
 * Please see
 */
#define ENABLE_SWING_UP 1						// Enable Pendulum Swing Up system
#define SWING_UP_CONTROL_CONFIG_DELAY 3000 		// Delay in cycles prior to switching from Swing Up controller to user selected controller
#define STAGE_0_AMP 200							// Swing Up Impulse amplitude for initial state
#define STAGE_1_AMP 130							// Swing Up Impulse amplitude for intermediate state
#define STAGE_2_AMP 120							// Swing Up Impulse amplitude for final state



/*
 * ENCODER ANGLE SLOPE CORRECTION compensates for any error introduced by a platform tilt relative to vertical.
 * The correction is computed over a time constant of greater than 100 seconds to avoid any distortion in measurements
 * that are conducted over shorter intervals of up to 10 seconds.
 */
#define ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION 	0		// Default 1 for system operation enable
#define OFFSET_FILTER_GAIN 						1		// Offset compensation gain value
#define LP_CORNER_FREQ_LONG_TERM 				0.01	// Corner frequency of low pass filter - default to 0.001
#define ENCODER_ANGLE_SLOPE_CORRECTION_SCALE 	200		// Set to 200
#define ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT	0	// Sets limit on operation time for slope angle correction
														// If set to zero, slope correction operates at all times
														// Default set to zero
/*
 * Rotor position limits are defined to limit rotor rotation to one full rotation in clockwise or
 * counterclockwise motion.
 */

#define ROTOR_POSITION_POSITIVE_LIMIT 240		// Maximum allowed rotation in positive angle in degrees
#define ROTOR_POSITION_NEGATIVE_LIMIT -240		// Minimum allowed rotation in negative angle in degrees

/*
 * Encoder position limits are defined to detect excursions in pendulum angle corresponding to
 * departure from control and to initiate control loop exit
 */

#define ENCODER_POSITION_POSITIVE_LIMIT  120		// Maximum allowed rotation in positive angle in steps
#define ENCODER_POSITION_NEGATIVE_LIMIT -120		// Minimum allowed rotation in negative angle in steps

#define ENABLE_TORQUE_CURRENT_ENTRY		0		    // Enables user input of torque current configuration in general mode
/*
 * Setting ENABLE_MOD_SIN_ROTOR_TRACKING to 1 enables a Rotor Position tracking command in the
 * form of an amplitude modulated sine wave signal
 * Frequency units and Rate are Hz
 * Amplitude units are steps
 *
 */

#define ENABLE_MOD_SIN_ROTOR_TRACKING 1		// If selected, disable all other modulation inputs
#define MOD_SIN_CARRIER_FREQ 0.15			// 0.15 default
#define MOD_SIN_START_CYCLES 5000			// Sine modulation starts at completion of angle calibration if enabled
#define MOD_SIN_AMPLITUDE 300				// 400 default
#define MOD_SIN_MODULATION_FREQ  0.02		// 0.02 default
#define MOD_SIN_MODULATION_MIN 0			// Default 0
/* Define for High Speed System */
#define MOD_SIN_SAMPLE_RATE (1/T_SAMPLE_DEFAULT)  // Equals system sample rate
#define ENABLE_SIN_MOD 1					// 1 default

#define ENABLE_ROTOR_CHIRP 0				// If selected, disable all other modulation inputs
#define ROTOR_CHIRP_START_FREQ 0.01			// 0.001 default
#define ROTOR_CHIRP_END_FREQ 15				// 5 default
#define ROTOR_CHIRP_PERIOD 20000			// 20000 default
#define ROTOR_CHIRP_SWEEP_DELAY 1000		// 0 default - enables control system to recover between sweeps
/* Define for High Speed System */
#define ROTOR_CHIRP_SAMPLE_RATE (1/T_SAMPLE_DEFAULT)  // Equals system sample rate
#define ROTOR_CHIRP_START_CYCLES 			// 0 default
#define ROTOR_CHIRP_STEP_AMPLITUDE 2  		// 0.3 default

#define ENABLE_ROTOR_TRACK_COMB_SIGNAL 0				// If selected, disable all other modulation inputs
#define ROTOR_TRACK_COMB_SIGNAL_SAMPLE_RATE 500.0		// 500.0 default for Low Speed
#define ROTOR_TRACK_COMB_SIGNAL_START_CYCLES 0			// 0 default
#define ROTOR_TRACK_COMB_SIGNAL_AMPLITUDE 5.0			// 0.05 default

#define ENABLE_DISTURBANCE_REJECTION_STEP 	1
#define LOAD_DISTURBANCE_SENSITIVITY_SCALE 	20			// Scale factor applied to increase measurement resolution for Load Disturbance Sensitivity Function

/*
 * Set ENABLE_ENCODER_TEST to 1 to enable a testing of encoder response for verification
 * occurring prior to control system start.
 *
 * This will be set to 0 and disabled for normal operation
 *
 */

#define ENABLE_ENCODER_TEST 0

/*
 * Set ENABLE_ROTOR_ACTUATOR_TEST to 1 to enable a testing of encoder response for verification
 * occurring prior to control system start.
 *
 * This will be set to 0 and disabled for normal operation
 *
 */

#define ENABLE_ROTOR_ACTUATOR_TEST 0
#define ROTOR_ACTUATOR_TEST_CYCLES 1

/*
 * Setting ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE = 1 applies a Rotor Position tracking
 * command input step signal
 */

#define ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE 1			// If selected, disable all other modulation inputs
#define ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE 20		// Default 8. Amplitude of step cycle. Note: Peak-to-Peak amplitude is double this value
#define ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL 16384 	// Default 10240
#define STEP_RESPONSE_AMP_LIMIT_ENABLE 0					// Enables limit of Step Response if rotor amplitude exceeds limit
															// Useful for protecting operation if summing step and sine drive
#define STEP_RESPONSE_AMP_LIMIT 350							// Angle limit for Step Response action
/*
 * Setting ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE = 1 applies a Rotor Position tracking
 * command input impulse signal
 */
#define ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE 0			// If selected, disable all other modulation inputs
#define ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE 8		// Amplitude of impulse in degrees
#define ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD 500 		// Duration of impulse in cycles
#define ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL 5000	    // Interval between impulse events in cycles
/* Define for High Speed System */
#define ROTOR_IMPULSE_SAMPLE_RATE (1/T_SAMPLE_DEFAULT)  		// Equals system sample rate							// Default sample rate
/*
 * Setting ENABLE_PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE = 1 applies a Pendulum Position tracking
 * command input impulse signal
 */
#define ENABLE_PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE 0		// If selected, disable all other modulation inputs
#define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE 500	// Amplitude of step cycle in steps equaling 75 degrees. Note: Peak-to-Peak amplitude is double this value
#define PENDULUM_POSITION_IMPULSE_AMPLITUDE_SCALE 4				// Amplitude scaling of impulse for Suspended and Inverted Mode
#define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD 2		// Duration of impulse in cycles
#define PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL 18000	// Interval between impulse events in cycles
/* Define for High Speed System */
#define PENDULUM_IMPULSE_SAMPLE_RATE (1/T_SAMPLE_DEFAULT)       // Equals system sample rate 						// Default sample rate

/*
 * DATA_REPORT_SPEED_SCALE enables reduced data rate for bandwidth constrained data acquisition systems
 */

#define DATA_REPORT_SPEED_SCALE 20


/*
 * Jawad Modifications
 */
#define COMMAND_SEPERATOR ','
#define COMMAND_END ';'


#define COMMAND_INDEX_SEND_GAMInput       0
#define COMMAND_INDEX_SEND_STATUS         1
#define COMMAND_INDEX_SEND_STATUS_INFO    10
#define COMMAND_INDEX_SEND_STATUS_ERROR   11



/*
 * UART DMA definitions
 */

#define UART_RX_BUFFER_SIZE 	(198)
#define SERIAL_MSG_MAXLEN 		(100)
#define SERIAL_MSG_EOF          '\r'

/*
 * Structure for the augmented floating-point PID Control.
 * This includes additional state associated with derivative filter
 *
 * This follows the ARM CMSIS architecture
 */

typedef struct
{
  float state_a[4];  /** The filter state array of length 4. */
  float Kp;          /** The proportional gain. */
  float Ki;          /** The integral gain. */
  float Kd;          /** The derivative gain. */
  float int_term;    /** The controller integral output */
  float control_output; /** The controller output */
} arm_pid_instance_a_f32;

#define delayUS_ASM(us) do {\
		asm volatile (	"MOV R0,%[loops]\n\t"\
				"1: \n\t"\
				"SUB R0, #1\n\t"\
				"CMP R0, #0\n\t"\
				"BNE 1b \n\t" : : [loops] "r" (16*us) : "memory"\
		);\
} while(0)

__STATIC_INLINE void DWT_Delay_until_cycle(volatile uint32_t cycle);

extern void pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float * current_error,
		float * sample_period, float * Deriv_Filt);

extern int encoder_position_read(int *encoder_position, int encoder_position_init, TIM_HandleTypeDef *htim3);
extern int rotor_position_read(int *rotor_position);

extern void set_mode_strings(void);
extern void get_user_mode_index(char * user_string, int * char_mode_select, int * mode_index, int * mode_interactive);
extern void rotor_position_set(void);
extern void select_mode_1(void);
extern void user_configuration(void);
extern void read_int(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, int * int_return);
extern void read_float(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, float *float_return);
extern void read_char(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, char * char_return);

extern void select_mode_1(void);
extern void user_configuration(void);
extern int Delay_Pulse();
extern void Main_StepClockHandler();
extern void apply_acceleration(float * acc, float* target_velocity_prescaled, float t_sample);

extern void user_prompt(void);
extern void rotor_actuator_high_speed_test(void);
extern void rotor_encoder_test(void);
extern void pendulum_system_id_test(void);
extern void motor_actuator_characterization_mode(void);
extern void interactive_rotor_actuator_control(void);

extern void user_configuration(void);
extern int mode_index_identification(char * user_config_input, int config_command_control, float *adjust_increment,
		arm_pid_instance_a_f32 *PID_Pend, arm_pid_instance_a_f32 *PID_Rotor);
extern void assign_mode_1(arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor);
extern void assign_mode_2(arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor);
extern void assign_mode_3(arm_pid_instance_a_f32 *PID_Pend,
		arm_pid_instance_a_f32 *PID_Rotor);

extern bool oppositeSigns(int x, int y);

extern volatile uint16_t gLastError;
/* Private function prototypes -----------------------------------------------*/
extern void MyFlagInterruptHandler(void);
extern void MX_TIM3_Init(void);
extern void MX_USART2_UART_Init(void);
extern void read_float(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, float *float_return);
extern void Error_Handler(uint16_t error);
extern void read_int(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, int * int_return);
extern void read_char(uint32_t * RxBuffer_ReadIdx, uint32_t * RxBuffer_WriteIdx , uint32_t * readBytes, char * char_return);
extern void select_mode_1(void);
extern void user_configuration(void);
extern int Delay_Pulse();
extern void Main_StepClockHandler();
extern void apply_acceleration(float * acc, float* target_velocity_prescaled, float t_sample);


/*
 * Timer 3, UART Transmit, and UART DMA Receive declarations
 */

extern TIM_HandleTypeDef htim3;

/*
  * Timer 3, UART Transmit, and UART DMA Receive declarations
  */

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

/*
 * UART Receive data structure
 */

typedef struct {
	uint32_t Len; /*!< Message length           */
	uint8_t Data[SERIAL_MSG_MAXLEN]; /*!< Message data             */
} T_Serial_Msg;

extern T_Serial_Msg Msg;

extern uint8_t RxBuffer[UART_RX_BUFFER_SIZE];
extern uint16_t Extract_Msg(uint8_t *CircularBuff, uint16_t StartPos, uint16_t LastPos,
		uint16_t BufMaxLen, T_Serial_Msg *Msg);

/* Acceleration control system variables */
extern volatile uint32_t apply_acc_start_time;
extern volatile uint32_t clock_int_time;
extern volatile uint32_t clock_int_tick;

/// PWM period variables used by step interrupt
extern volatile uint32_t desired_pwm_period;
extern volatile uint32_t current_pwm_period;

extern float target_velocity_prescaled;
extern int32_t enable_speed_prescale;

/* System data reporting */
extern char tmp_string[256];
extern char msg[192];
extern char msg_pad[64];
extern char test_msg[128];

/* System timing variables */

extern uint32_t tick, tick_cycle_current, tick_cycle_previous, tick_cycle_start,
       tick_read_cycle, tick_read_cycle_start,tick_wait_start,tick_wait;

extern volatile uint32_t current_cpu_cycle, prev_cpu_cycle, last_cpu_cycle, target_cpu_cycle, prev_target_cpu_cycle;
extern volatile int current_cpu_cycle_delay_relative_report;

uint32_t t_sample_cpu_cycles;
extern float Tsample, Tsample_rotor, test_time;
extern float angle_scale;
extern int enable_high_speed_sampling;

/* Reset state tracking */
extern int reset_state;

/* Motor configuration */
extern uint16_t min_speed, max_speed, max_accel, max_decel;

/* Serial interface variables */
extern uint32_t RxBuffer_ReadIdx;
extern uint32_t RxBuffer_WriteIdx;
extern uint32_t readBytes;

/* Control system output signal */
extern float rotor_control_target_steps;
extern float rotor_control_target_steps_curr;
extern float rotor_control_target_steps_prev;

/* Control system variables */
extern int rotor_position_delta;
extern int initial_rotor_position;
extern int cycle_count;
extern int i, j, k, m;
extern int ret;

/* PID control system variables */
extern float windup, rotor_windup;
extern float *current_error_steps, *current_error_rotor_steps;
extern float *sample_period, *sample_period_rotor;

/* Loop timing measurement variables */
extern int cycle_period_start;
extern int cycle_period_sum;
extern int enable_cycle_delay_warning;

/* PID control variables */
extern float *deriv_lp_corner_f;
extern float *deriv_lp_corner_f_rotor;
extern float proportional, rotor_p_gain;
extern float integral, rotor_i_gain;
extern float derivative, rotor_d_gain;

/* State Feedback variables */
extern int enable_state_feedback;
extern float integral_compensator_gain;
extern float feedforward_gain;
extern float current_error_rotor_integral;

/* Reference tracking command */
extern float reference_tracking_command;

/* Pendulum position and tracking command */

/* Rotor position and tracking command */
extern int rotor_position_steps;
extern float rotor_position_command_steps;
extern float rotor_position_command_steps_pf, rotor_position_command_steps_pf_prev;
extern float rotor_position_command_deg;
extern float rotor_position_steps_prev, rotor_position_filter_steps, rotor_position_filter_steps_prev;
extern float rotor_position_diff, rotor_position_diff_prev;
extern float rotor_position_diff_filter, rotor_position_diff_filter_prev;
extern int rotor_target_in_steps;
extern int initial_rotor_position;

/* Rotor Plant Design variables */
extern int select_rotor_plant_design, enable_rotor_plant_design, enable_rotor_plant_gain_design;
extern int rotor_control_target_steps_int;
extern float rotor_damping_coefficient, rotor_natural_frequency;
extern float rotor_plant_gain;
extern float rotor_control_target_steps_gain;
extern float rotor_control_target_steps_filter_2, rotor_control_target_steps_filter_prev_2;
extern float rotor_control_target_steps_prev_prev, rotor_control_target_steps_filter_prev_prev_2;
extern float c0, c1, c2, c3, c4, ao, Wn2;
extern float fo_r, Wo_r, IWon_r, iir_0_r, iir_1_r, iir_2_r;

/* Encoder position variables */
extern uint32_t cnt3;
extern int range_error;
extern float encoder_position;
extern int encoder_position_steps;
extern int encoder_position_init;
extern int previous_encoder_position;
extern int max_encoder_position;
extern int global_max_encoder_position;
extern int prev_global_max_encoder_position;
extern int encoder_position_down;
extern int encoder_position_curr;
extern int encoder_position_prev;

/* Angle calibration variables */
extern float encoder_position_offset;
extern float encoder_position_offset_zero;
extern int enable_angle_cal;
extern int enable_angle_cal_resp;
extern int offset_end_state;
extern int offset_start_index;
extern int angle_index;
extern int angle_avg_index;
extern int angle_avg_span;
extern int offset_angle[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
extern float encoder_position_offset_avg[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
extern int angle_cal_end;
extern int angle_cal_complete;

/* Swing Up system variables */
extern int enable_swing_up;
extern int enable_swing_up_resp;
extern bool peaked;
extern bool handled_peak;
extern int zero_crossed;
extern motorDir_t swing_up_direction;
extern int swing_up_state, swing_up_state_prev;
extern int stage_count;
extern int stage_amp;

/* Initial control state parameter storage */
extern float init_r_p_gain, init_r_i_gain, init_r_d_gain;
extern float init_p_p_gain, init_p_i_gain, init_p_d_gain;
extern int init_enable_state_feedback;
extern float init_integral_compensator_gain;
extern float init_feedforward_gain;
extern int init_enable_state_feedback;
extern int init_enable_disturbance_rejection_step;
extern int init_enable_sensitivity_fnc_step;
extern int init_enable_noise_rejection_step;
extern int init_enable_rotor_plant_design;
extern int init_enable_rotor_plant_gain_design;


/* Low pass filter variables */
extern float fo, Wo, IWon, iir_0, iir_1, iir_2;
extern float fo_LT, Wo_LT, IWon_LT;
extern float iir_LT_0, iir_LT_1, iir_LT_2;
extern float fo_s, Wo_s, IWon_s, iir_0_s, iir_1_s, iir_2_s;

/* Slope correction system variables */
extern int slope;
extern int slope_prev;
extern float encoder_angle_slope_corr_steps;

/* Adaptive control variables */
extern float adaptive_error, adaptive_threshold_low, adaptive_threshold_high;
extern float error_sum_prev, error_sum, error_sum_filter_prev, error_sum_filter;
extern int adaptive_entry_tick, adaptive_dwell_period;
extern int enable_adaptive_mode, adaptive_state, adaptive_state_change;
extern float rotor_position_command_steps_prev;

/* Rotor impulse variables */
extern int rotor_position_step_polarity;
extern int impulse_start_index;

/* User configuration variables */
extern int clear_input;
uint32_t enable_control_action;
extern int max_speed_read, min_speed_read;
extern int select_suspended_mode;
extern int motor_response_model;
extern int enable_rotor_actuator_test, enable_rotor_actuator_control;
extern int enable_encoder_test;
extern int enable_rotor_actuator_high_speed_test;
extern int enable_motor_actuator_characterization_mode;
extern int motor_state;
extern float torq_current_val;


/* Rotor chirp system variables */
extern int enable_rotor_chirp;
extern int chirp_cycle;
extern int chirp_dwell_cycle;
extern float chirp_time;
extern float rotor_chirp_start_freq;
extern float rotor_chirp_end_freq;
extern float rotor_chirp_period ;
extern float rotor_chirp_frequency;
extern float rotor_chirp_amplitude;
extern int rotor_chirp_step_period;

extern float pendulum_position_command_steps;

/* Modulates sine tracking signal system variables */
extern int enable_mod_sin_rotor_tracking;
extern int enable_rotor_position_step_response_cycle;
extern int disable_mod_sin_rotor_tracking;
extern int sine_drive_transition;
extern float mod_sin_amplitude;
extern float rotor_control_sin_amplitude;
extern float rotor_sine_drive, rotor_sine_drive_mod;
extern float rotor_mod_control;
extern float mod_sin_carrier_frequency;

/* Pendulum impulse system variables */
extern int enable_pendulum_position_impulse_response_cycle;

/* Rotor high speed test system variables */
extern int swing_cycles, rotor_test_speed_min, rotor_test_speed_max;
extern int rotor_test_acceleration_max, swing_deceleration_max;
extern int start_angle_a[20], end_angle_a[20], motion_dwell_a[20];
extern int abs_encoder_position_prior, abs_encoder_position_after, abs_encoder_position_max;
extern uint16_t current_speed;

/*Pendulum system ID variable */
extern int enable_pendulum_sysid_test;

/* Full system identification variables */
extern int enable_full_sysid;
extern float full_sysid_max_vel_amplitude_deg_per_s;
extern float full_sysid_min_freq_hz;
extern float full_sysid_max_freq_hz;
extern int full_sysid_num_freqs;
extern float full_sysid_freq_log_step;
extern int full_sysid_start_index;

/* Rotor comb drive system variables */
extern int enable_rotor_tracking_comb_signal;
extern float rotor_track_comb_signal_frequency;
extern float rotor_track_comb_command;
extern float rotor_track_comb_amplitude;

/* Sensitivity function system variables */
extern int enable_disturbance_rejection_step;
extern int enable_noise_rejection_step;
extern int enable_plant_rejection_step;
extern int enable_sensitivity_fnc_step;
extern float load_disturbance_sensitivity_scale;



/* Noise rejection sensitivity function low pass filter */

extern float noise_rej_signal_filter, noise_rej_signal;
extern float noise_rej_signal_prev, noise_rej_signal_filter_prev;

/*
 * Real time user input system variables
 */

extern char config_message[16];
extern int config_command;
extern int display_parameter;
extern int step_size;
extern float adjust_increment;
extern int mode_index;

/* Real time data reporting index */
extern int report_mode;
extern int speed_scale;
extern int speed_governor;

/*
 * User selection mode values
 */

extern int mode_1;				// Enable LQR Motor Model M
extern int mode_2;				// Enable LRR Motor Model H
extern int mode_3;				// Enable LQR Motor Model L
extern int mode_4;				// Enable Suspended Mode Motor Model M
extern int mode_5;				// Enable sin drive track signal
extern int mode_adaptive_off;	// Disable adaptive control
extern int mode_adaptive;		// Enable adaptive control
extern int mode_8;				// Enable custom configuration entry
extern int mode_9;				// Disable sin drive track signal
extern int mode_10;			// Enable Single PID Mode with Motor Model M
extern int mode_11;			// Enable rotor actuator and encoder test mode
extern int mode_13;			// Enable rotor control system evaluation
extern int mode_15;			// Enable interactive control of rotor actuator
extern int mode_16;			// Enable load disturbance function step mode
extern int mode_17;			// Enable noise disturbance function step mode
extern int mode_18;			// Enable sensitivity function step mode
extern int mode_19;            // Enable full system identification mode
extern int mode_quit;			// Initiate exit from control loop
extern int mode_interactive;	// Enable continued terminal interactive user session
extern int mode_index_prev, mode_index_command;
extern int mode_transition_tick;
extern int mode_transition_state;
extern int transition_to_adaptive_mode;


/*
 * Real time user input characters
 */

extern char mode_string_stop[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_1[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_2[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_3[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_4[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_8[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_5[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_accel[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_accel[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_amp[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_amp[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_single_pid[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_test[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_control[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_motor_characterization_mode[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_load_dist[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_load_dist_step[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_noise_dist_step[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_plant_dist_step[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_full_sysid[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_pend_p[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_pend_p[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_pend_i[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_pend_i[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_pend_d[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_pend_d[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_rotor_p[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_rotor_p[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_rotor_i[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_rotor_i[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_rotor_d[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_rotor_d[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_torq_c[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_torq_c[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_max_s[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_max_s[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_min_s[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_min_s[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_max_a[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_max_a[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_max_d[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_max_d[UART_RX_BUFFER_SIZE];
extern char mode_string_enable_step[UART_RX_BUFFER_SIZE];
extern char mode_string_disable_step[UART_RX_BUFFER_SIZE];
extern char mode_string_enable_pendulum_impulse[UART_RX_BUFFER_SIZE];
extern char mode_string_disable_pendulum_impulse[UART_RX_BUFFER_SIZE];
extern char mode_string_enable_load_dist[UART_RX_BUFFER_SIZE];
extern char mode_string_disable_load_dist[UART_RX_BUFFER_SIZE];
extern char mode_string_enable_noise_rej_step[UART_RX_BUFFER_SIZE];
extern char mode_string_disable_noise_rej_step[UART_RX_BUFFER_SIZE];
extern char mode_string_disable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
extern char mode_string_enable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_step_size[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_step_size[UART_RX_BUFFER_SIZE];
extern char mode_string_select_mode_5[UART_RX_BUFFER_SIZE];
extern char mode_string_enable_high_speed_sampling[UART_RX_BUFFER_SIZE];
extern char mode_string_disable_high_speed_sampling[UART_RX_BUFFER_SIZE];
extern char mode_string_enable_speed_prescale[UART_RX_BUFFER_SIZE];
extern char mode_string_disable_speed_prescale[UART_RX_BUFFER_SIZE];
extern char mode_string_disable_speed_governor[UART_RX_BUFFER_SIZE];
extern char mode_string_enable_speed_governor[UART_RX_BUFFER_SIZE];
extern char mode_string_reset_system[UART_RX_BUFFER_SIZE];


extern int char_mode_select;	// Flag detecting whether character mode select entered


extern char message_received[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_1[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_2[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_3[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_4[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_5[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_8[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_single_pid[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_test[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_control[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_high_speed_test[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_motor_characterization_mode[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_pendulum_sysid_test[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_accel[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_accel[UART_RX_BUFFER_SIZE];
extern char mode_string_inc_amp[UART_RX_BUFFER_SIZE];
extern char mode_string_dec_amp[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_load_dist_step[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_noise_dist_step[UART_RX_BUFFER_SIZE];
extern char mode_string_mode_plant_dist_step[UART_RX_BUFFER_SIZE];
extern char mode_string_stop[UART_RX_BUFFER_SIZE];

/* CMSIS Variables */
arm_pid_instance_a_f32 PID_Pend, PID_Rotor;
extern float Deriv_Filt_Pend[2];
extern float Deriv_Filt_Rotor[2];
extern float Wo_t, fo_t, IWon_t;

/* System timing variables */

extern uint32_t tick, tick_cycle_current, tick_cycle_previous, tick_cycle_start,
       tick_read_cycle, tick_read_cycle_start,tick_wait_start,tick_wait;

extern volatile uint32_t current_cpu_cycle, prev_cpu_cycle, last_cpu_cycle, target_cpu_cycle, prev_target_cpu_cycle;
extern volatile int current_cpu_cycle_delay_relative_report;

extern uint32_t t_sample_cpu_cycles;
extern float Tsample, Tsample_rotor, test_time;
extern float angle_scale;
extern int enable_high_speed_sampling;

/* Reset state tracking */
extern int reset_state;

/* Motor configuration */
extern uint16_t min_speed, max_speed, max_accel, max_decel;

/* Serial interface variables */
extern uint32_t RxBuffer_ReadIdx;
extern uint32_t RxBuffer_WriteIdx;
extern uint32_t readBytes;


#endif /* __EDUKIT_H */
