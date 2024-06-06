#include "InvertedPendulumGAM.h"
#include "AdvancedErrorManagement.h" 
#include "GAMSignalChecker.h"
#include <string.h>
#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <assert.h>

using namespace MARTe;

namespace MFI {

InvertedPendulumGAM::InvertedPendulumGAM() : GAM() {

   //INPUT_encoder_position  = NULL_PTR(float32*);
   INPUT_rotor_position_steps    = NULL_PTR(int32*);
   INPUT_L6474_Board_Pwm1Counter = NULL_PTR(uint32*);
   //INPUT_CYCCNT                  = NULL_PTR(uint32*);
   INPUT_message_count           = NULL_PTR(uint32*);
   //INPUT_state                   = NULL_PTR(uint8*);
   INPUT_encoder_counter        = NULL_PTR(uint32*);
   INPUT_break_Control_Loop     = NULL_PTR(uint8*);

   OUTPUT_rotor_control_target_steps    = NULL_PTR(int32*);
   OUTPUT_gpioState                     = NULL_PTR(uint8*);
   OUTPUT_L6474_Board_Pwm1Period        = NULL_PTR(uint32*);
   OUTPUT_break_Control_Loop            = NULL_PTR(uint8*);
   OUTPUT_state                         = NULL_PTR(uint8*);
   OUTPUT_encoder_position              = NULL_PTR(float32*);

}

InvertedPendulumGAM::~InvertedPendulumGAM() {

}

// void InvertedPendulumGAM::show_error( bool isError = false){
//      REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
// }

void InvertedPendulumGAM::control_logic_Initialise() {
    /* Initialize reset state indicating that reset has occurred */

    state = STATE_INITIALIZATION;

	reset_state = 1;

	/* initialize Integrator Mode time variables */
	apply_acc_start_time = 0u;
	clock_int_time = 0u;
	clock_int_tick = 0u;

	/* Initialize PWM period variables used by step interrupt */
	desired_pwm_period = 0u;
	current_pwm_period = 0u;
	target_velocity_prescaled = 0;

	/* Initialize default start mode and reporting mode */
	mode_index = 1;
	report_mode = 1;

	/*Initialize serial read variables */
	RxBuffer_ReadIdx = 0u;
	RxBuffer_WriteIdx = 0u;
	readBytes = 0u;

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


	/* Default select_suspended_mode */
	select_suspended_mode = ENABLE_SUSPENDED_PENDULUM_CONTROL;

	
	/* Default Starting Control Configuration */
	max_accel = (u_int16_t) MAX_ACCEL;
	max_decel = (u_int16_t) MAX_DECEL;
	max_speed = (u_int16_t) MAX_SPEED_MODE_1;
	min_speed = (u_int16_t) MIN_SPEED_MODE_1;
	
    /* Default torque current */
    /* ##########################**** MODIFY ****##################################
	torq_current_val = MAX_TORQUE_CONFIG;
	L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
    ###########################################################################################*/
	

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

	/* Disable adaptive_mode by default */
	enable_adaptive_mode = 0;

	
	/* Controller structure and variable allocation */
	current_error_steps = (float*)malloc(sizeof(float));
	if (current_error_steps == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
        REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}
	current_error_rotor_steps = (float*)malloc(sizeof(float));
	if (current_error_rotor_steps == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
		REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}
	sample_period = (float*)malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
		REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}
	deriv_lp_corner_f = (float*)malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
		REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}
	deriv_lp_corner_f_rotor = (float*)malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
		REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}
	sample_period_rotor = (float*)malloc(sizeof(float));
	if (sample_period == NULL) {
		sprintf(msg_display, "Memory allocation error\r\n");
		REPORT_ERROR(ErrorManagement::ParametersError, msg_display);
	}

	
	/* Configure controller filter and sample time parameters */
	*deriv_lp_corner_f = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	*deriv_lp_corner_f_rotor = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	t_sample_cpu_cycles = (u_int32_t) round(T_SAMPLE_DEFAULT * RCC_HCLK_FREQ);
	Tsample = (float) t_sample_cpu_cycles / RCC_HCLK_FREQ;
	*sample_period = Tsample;
	Tsample_rotor = Tsample;
	*sample_period_rotor = Tsample_rotor;

	/* PID Derivative Low Pass Filter Coefficients */

	fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	Wo_t = 2 * 3.141592654 * fo_t;
	IWon_t = 2 / (Wo_t * (*sample_period));
	Deriv_Filt_Pend[0] = 1 / (1 + IWon_t);
	Deriv_Filt_Pend[1] = Deriv_Filt_Pend[0] * (1 - IWon_t);

	fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	Wo_t = 2 * 3.141592654 * fo_t;
	IWon_t = 2 / (Wo_t * (*sample_period));
	Deriv_Filt_Rotor[0] = 1 / (1 + IWon_t);
	Deriv_Filt_Rotor[1] = Deriv_Filt_Rotor[0] * (1 - IWon_t);

	
	/* Configure primary controller parameters */
	windup = PRIMARY_WINDUP_LIMIT;

	/* Configure secondary Rotor controller parameters */
	rotor_windup = SECONDARY_WINDUP_LIMIT;

	/* Compute Low Pass Filter Coefficients for Rotor Position filter and Encoder Angle Slope Correction */
	fo = LP_CORNER_FREQ_ROTOR;
	Wo = 2 * 3.141592654 * fo;
	IWon = 2 / (Wo * Tsample);
	iir_0 = 1 / (1 + IWon);
	iir_1 = iir_0;
	iir_2 = iir_0 * (1 - IWon);
	fo_s = LP_CORNER_FREQ_STEP;
	Wo_s = 2 * 3.141592654 * fo_s;
	IWon_s = 2 / (Wo_s * Tsample);
	iir_0_s = 1 / (1 + IWon_s);
	iir_1_s = iir_0_s;
	iir_2_s = iir_0_s * (1 - IWon_s);
	fo_LT = LP_CORNER_FREQ_LONG_TERM;
	Wo_LT = 2 * 3.141592654 * fo_LT;
	IWon_LT = 2 / (Wo_LT * Tsample);
	iir_LT_0 = 1 / (1 + IWon_LT);
	iir_LT_1 = iir_LT_0;
	iir_LT_2 = iir_LT_0 * (1 - IWon_LT);
	/*
	 * Request user input for mode configuration
	 */

	enable_adaptive_mode = ENABLE_ADAPTIVE_MODE;
	adaptive_threshold_low = ADAPTIVE_THRESHOLD_LOW;
	adaptive_threshold_high = ADAPTIVE_THRESHOLD_HIGH;
	adaptive_state = ADAPTIVE_STATE;
	adaptive_state_change = 0;
	adaptive_dwell_period = ADAPTIVE_DWELL_PERIOD;

}

void InvertedPendulumGAM::control_logic_State_Initialization(){

    // mode_interactive = 0;
    // user_prompt();

    // /*
    //     * If user has responded to previous query for configuration, then system remains in interactive mode
    //     * and default state is not automatically enabled
    //     */


    // if (mode_interactive == 0) {
    //     sprintf(msg_display, "\n\rEnter Mode Selection Now or System Will Start in Default Mode in 5 Seconds..: ");
    // }

    /*
        * If user has responded to query for configuration, then system remains in interactive mode
        * and default state is not automatically enabled
        */

    // if (mode_interactive == 1) {
    //     sprintf(msg_display, "\n\rEnter Mode Selection Now: \n\r");
    // }

    /* Start timer for configuration command read loop */
    // tick_read_cycle_start = HAL_GetTick();
    // /* Configuration command read loop */
     user_configuration();


    /* Report configuration values */
    // if (ACCEL_CONTROL == 0){
    //     sprintf(msg_display, "\n\rMotor Profile Speeds Set at Min %u Max %u Steps per Second", min_speed, max_speed);
    // }
    // if (select_suspended_mode == 0){
    //     sprintf(msg_display, "\n\rInverted Pendulum Mode Selected");
    // }
    // if (select_suspended_mode == 1){
    //     sprintf(msg_display, "\n\rSuspended Pendulum Mode Selected");
    // }

    // sprintf(msg_display, "\n\rMotor Torque Current Set at %0.1f mA", torq_current_val);
  

    // /* Motor Control Characterization Test*/
    // if (enable_motor_actuator_characterization_mode == 1) {
    //     motor_actuator_characterization_mode();
    // }
    // /* Interactive digital motor control system */
    // if (enable_rotor_actuator_control == 1) {
    //     interactive_rotor_actuator_control();
    // }

    /*
        * 	Rotor and Encoder Test Sequence will execute by moving rotor and reportin angle values
        * 	as well as requesting pendulum motion followed by reporting of pendulum angles
        *
        * 	Agreement between actions and reported values confirms proper installation of actuator
        * 	and pendulum encoder.
        *
        */

    // if (enable_rotor_actuator_test == 1) {
    //     rotor_encoder_test();
    // }

    /*
        * Configure Primary and Secondary PID controller data structures
        * Scale by CONTROLLER_GAIN_SCALE set to default value of unity
        */


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

    PID_Pend.state_a[0] = 0;
    PID_Pend.state_a[1] = 0;
    PID_Pend.state_a[2] = 0;
    PID_Pend.state_a[3] = 0;

    PID_Rotor.state_a[0] = 0;
    PID_Rotor.state_a[1] = 0;
    PID_Rotor.state_a[2] = 0;
    PID_Rotor.state_a[3] = 0;

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
    sprintf(msg_display, "\n\rEnable Design: %i iir_0 %0.4f iir_1 %0.4f iir_2 %0.4f\n\r", enable_rotor_plant_design, iir_0_r, iir_1_r, iir_2_r);
    HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);


    //Optional display coefficients for rotor plant design transfer function
    sprintf(msg_display,
            "\n\ra0 %0.4f c0 %0.4f c1 %0.4f c2 %0.4f c3 %0.4f c4 %0.4f\n\r", ao, c0, c1, c2, c3, c4);
    HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);

        */


    /*
        * *************************************************************************************************
        *
        * Control System Initialization Sequence
        *
        * *************************************************************************************************
        */

    /* Setting enable_control_action enables control loop */
    enable_control_action = (u_int32_t)ENABLE_CONTROL_ACTION;

    /*
        * Set Motor Position Zero occuring only once after reset and suppressed thereafter
        * to maintain angle calibration
        */

    if (reset_state == 1){
        //rotor_position_set();
    }

    // ret = rotor_position_read(&rotor_position_steps);
    //     sprintf(msg_display,  "\r\nPrepare for Control Start - Initial Rotor Position: %i\r\n",
    //         rotor_position_steps);


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

    // BSP_MotorControl_GoTo(0, 3);
    // BSP_MotorControl_WaitWhileActive(0);
    // HAL_Delay(150);
    // BSP_MotorControl_GoTo(0, -3);
    // BSP_MotorControl_WaitWhileActive(0);
    // HAL_Delay(150);
    // BSP_MotorControl_GoTo(0, 3);
    // BSP_MotorControl_WaitWhileActive(0);
    // HAL_Delay(150);
    // BSP_MotorControl_GoTo(0, 0);
    // BSP_MotorControl_WaitWhileActive(0);

    // sprintf(msg_display, "Test for Pendulum at Rest - Waiting for Pendulum to Stabilize\r\n");
    // HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);

    // encoder_position_init = 0;
    // ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
    // encoder_position_prev = encoder_position_steps;
    // HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
    // ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
    // encoder_position_curr = encoder_position_steps;
    // while (encoder_position_curr != encoder_position_prev) {
    //     ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
    //     encoder_position_prev = encoder_position_steps;
    //     HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
    //     ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
    //     encoder_position_curr = encoder_position_steps;

    //     /*
    //         * Ensure stability reached with final motion test
    //         */

    //     if (encoder_position_prev == encoder_position_curr) {
    //         HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
    //         ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
    //         encoder_position_prev = encoder_position_steps;
    //         HAL_Delay(INITIAL_PENDULUM_MOTION_TEST_DELAY);
    //         ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
    //         encoder_position_curr = encoder_position_steps;
    //         if (encoder_position_prev == encoder_position_curr) {
    //             break;
    //         }
    //     }
    //     /* Alert user of undesired motion */
    //     sprintf(msg_display, "Pendulum Motion Detected with angle %0.2f - Waiting for Pendulum to Stabilize\r\n",
    //             (float) ((encoder_position_curr - encoder_position_prev)
    //                     / ENCODER_READ_ANGLE_SCALE));
    //     HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display),
    //             HAL_MAX_DELAY);
    // }

    // sprintf(msg_display, "Pendulum Now at Rest and Measuring Pendulum Down Angle\r\n");
    // HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);

    // /* Calibrate down angle */

    // /*
    //     * Initialize Pendulum Angle Read offset by setting encoder_position_init
    //     */

    // HAL_Delay(100);
    // ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
    // encoder_position_init = encoder_position_steps;

    // if (ret == -1) {
    //     sprintf(msg_display, "Encoder Position Under Range Error\r\n");
    //     HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display),
    //             HAL_MAX_DELAY);
    // }
    // if (ret == 1) {
    //     sprintf(msg_display, "Encoder Position Over Range Error\r\n");
    //     HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display),
    //             HAL_MAX_DELAY);
    // }

    // ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
    // encoder_position_down = encoder_position_steps;
    // sprintf(msg_display, "Pendulum Initial Angle %i\r\n", encoder_position_steps);
    // HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);


    // if (enable_swing_up == 0){
    //     /*
    //         * Alert user with rotor motion prompt to adjust pendulum upright by
    //         */
    //     BSP_MotorControl_GoTo(0, 30);
    //     BSP_MotorControl_WaitWhileActive(0);
    //     HAL_Delay(150);
    //     BSP_MotorControl_GoTo(0, -30);
    //     BSP_MotorControl_WaitWhileActive(0);
    //     HAL_Delay(150);
    //     BSP_MotorControl_GoTo(0, 30);
    //     BSP_MotorControl_WaitWhileActive(0);
    //     HAL_Delay(150);
    //     BSP_MotorControl_GoTo(0, 0);
    //     BSP_MotorControl_WaitWhileActive(0);

    //     /* Request user action to bring pendulum upright */
    //     if(select_suspended_mode == 0){
    //         sprintf(msg_display,
    //                 "Adjust Pendulum Upright By Turning CCW Control Will Start When Vertical\r\n");
    //         HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);
    //     }

    // }

    // /*
    //     * Detect Start Condition for Pendulum Angle for Inverted Model
    //     *
    //     * Detect Pendulum Angle equal to vertical within tolerance of START_ANGLE
    //     *
    //     * Exit if no vertical orientation action detected and alert user to restart,
    //     * then disable control and enable system restart.
    //     *
    //     * Permitted delay for user action is PENDULUM_ORIENTATION_START_DELAY.
    //     *
    //     */

    // /*
    //     * System start option with manual lifting of Pendulum to vertical by user
    //     */

    // if (enable_swing_up == 0){

    //     tick_wait_start = HAL_GetTick();
    //     if (select_suspended_mode == 0) {
    //         while (1){
    //             ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
    //             if (fabs(encoder_position_steps - encoder_position_down - (int) (180 * angle_scale)) < START_ANGLE * angle_scale){
    //                 HAL_Delay(START_ANGLE_DELAY);
    //                 break;
    //             }
    //             if (fabs(encoder_position_steps - encoder_position_down + (int)(180 * angle_scale)) < START_ANGLE * angle_scale){
    //                 encoder_position_down = encoder_position_down - 2*(int)(180 * angle_scale);
    //                 HAL_Delay(START_ANGLE_DELAY);
    //                 break;
    //             }
    //             tick_wait = HAL_GetTick();

    //             if ( (tick_wait - tick_wait_start) > PENDULUM_ORIENTATION_START_DELAY){
    //                 sprintf(msg_display, "Pendulum Upright Action Not Detected - Restarting ...\r\n");
    //                 HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);
    //                 enable_control_action = 0;
    //                 break;
    //             }
    //         }
    //     }
    // }


    /*
        * For case of Suspended Mode Operation, no initial condition check is required
        * for pendulum down angle.
        */

    // if(select_suspended_mode == 1){
    //     sprintf(msg_display, "Suspended Mode Control Will Start in %i Seconds\r\n",
    //             (int) (CONTROL_START_DELAY / 1000));
    //     HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);
    // }


    /*
        * Initialize Primary and Secondary PID controllers
        */

    
}

void InvertedPendulumGAM::control_logic_State_SwingingUp_Prepare() {
    /*
        * Apply controller parameters for initial operation at completion of
        * Swing Up
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

    /* Initialize Pendulum PID control state */
    pid_filter_control_execute(&PID_Pend, current_error_steps, sample_period, Deriv_Filt_Pend);

    /* Initialize Rotor PID control state */
    *current_error_rotor_steps = 0;
    pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, sample_period_rotor, Deriv_Filt_Rotor);

    /* Initialize control system variables */

    cycle_count = CYCLE_LIMIT;
    i = 0;
    //rotor_position_steps = 0;
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
    // tick_cycle_start = HAL_GetTick();
    // tick_cycle_previous = tick_cycle_start;
    // tick_cycle_current =  tick_cycle_start;
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

    // /* Clear read buffer */
    // for (k = 0; k < SERIAL_MSG_MAXLEN; k++) {
    //     Msg.Data[k] = 0;
    // }
    // /* Initialize UART receive system */
    // __HAL_DMA_RESET_HANDLE_STATE(&hdma_usart2_rx);

    /*
        * Record user selected operation variable values.  Values will be
        * restored after Swing Up completion or after Angle Calibration
        * completion
        */

    /* Initial control state parameter storage */

    init_r_p_gain = PID_Rotor.Kp;
    init_r_i_gain = PID_Rotor.Ki;
    init_r_d_gain = PID_Rotor.Kd;
    init_p_p_gain = PID_Pend.Kp;
    init_p_i_gain = PID_Pend.Ki;
    init_p_d_gain = PID_Pend.Kd;
    init_enable_state_feedback = enable_state_feedback;
    init_integral_compensator_gain = integral_compensator_gain;
    init_feedforward_gain = feedforward_gain;
    //int init_enable_state_feedback = enable_state_feedback;
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

        // /* Set Torque Current value to 800 mA (normal operation will revert to 400 mA */
        // torq_current_val = MAX_TORQUE_SWING_UP;
        // L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);

        // sprintf(msg_display, "Pendulum Swing Up Starting\r\n");
        // HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);

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
        //sprintf(msg_display,"Current Position %0.2f\r\n", (encoder_position - encoder_position_down)/angle_scale);
        //HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);

        //sprintf(msg_display,"Current Position Down %0.2f\r\n", encoder_position_down/angle_scale);
        //HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);

        // /* Initiate first swing */
        swing_up_direction = FORWARD;
        // BSP_MotorControl_Move(0, swing_up_direction, 150);
        // BSP_MotorControl_WaitWhileActive(0);


        /* Enter Swing Up Loop */
        // while (1)
        // {
        //     HAL_Delay(2);
        //     ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
        //     /* Optional Swing Up progress reporting */
        //     //sprintf(msg_display,"Rotor Impulse Amplitude %i Max Angle (degrees) %0.3f\r\n", stage_amp, fabs((float)(global_max_encoder_position)/(ENCODER_READ_ANGLE_SCALE)));
        //     //HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display, strlen(msg_display), HAL_MAX_DELAY);

        //     /* Break if pendulum angle relative to vertical meets tolerance (for clockwise or counter clockwise approach */
        //     if (fabs(encoder_position_steps - encoder_position_down - (int) (180 * angle_scale)) < START_ANGLE * angle_scale){
        //         break;
        //     }
        //     if (fabs(encoder_position_steps - encoder_position_down + (int)(180 * angle_scale)) < START_ANGLE * angle_scale){
        //         encoder_position_down = encoder_position_down - 2*(int)(180 * angle_scale);
        //         break;
        //     }

        //     if (zero_crossed)
        //     {
        //         zero_crossed = 0;
        //         // Push it aka put some more kinetic energy into the pendulum
        //         if (swing_up_state == 0){
        //             BSP_MotorControl_Move(0, swing_up_direction, stage_amp);
        //             BSP_MotorControl_WaitWhileActive(0);
        //             stage_count++;

        //             if (prev_global_max_encoder_position != global_max_encoder_position && stage_count > 4){
        //             if (abs(global_max_encoder_position) < 600){
        //                 stage_amp = STAGE_0_AMP;
        //             }
        //             if (abs(global_max_encoder_position) >= 600 && abs(global_max_encoder_position) < 1000){
        //                 stage_amp = STAGE_1_AMP;
        //             }
        //             if (abs(global_max_encoder_position) >= 1000){
        //                 stage_amp = STAGE_2_AMP;
        //             }
        //             }
        //             prev_global_max_encoder_position = global_max_encoder_position;
        //             global_max_encoder_position = 0;
        //             ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
        //         }
        //     }


        //     // We have a peak but did not handle it yet
        //     if (peaked && !handled_peak)
        //     {
        //         // Ensure we only enter this branch one per peak
        //         handled_peak = 1;
        //         // Reset maximum encoder value to reassess after crossing the bottom
        //         max_encoder_position = 0;
        //         // Switch motor direction
        //         swing_up_direction = swing_up_direction == FORWARD ? BACKWARD : FORWARD;
        //     }
        // }
    }

    /*
        * *************************************************************************************************
        *
        * Control Loop Start
        *
        * *************************************************************************************************
        */

}

void InvertedPendulumGAM::control_logic_State_PendulumStablisation_Prepare(){
    encoder_position_init = 0;
    encoder_position_prev=-1;
    control_logic_State_PendulumStablisation_testCount = 0;
    control_logic_State_PendulumStablisation_isSecondRead = false;
}

void InvertedPendulumGAM::control_logic_State_Main_Prepare(){
       // enable_control_action = 1u;

    if (ACCEL_CONTROL == 1) {
        // BSP_MotorControl_HardStop(0);
        // L6474_CmdEnable(0);
        target_velocity_prescaled = 0;
        // L6474_Board_SetDirectionGpio(0, BACKWARD);
    }

    /*
        * Set Torque Current to value for normal operation
        */
    // torq_current_val = MAX_TORQUE_CONFIG;
    // L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);

    // target_cpu_cycle = DWT->CYCCNT;
    // prev_cpu_cycle = DWT->CYCCNT;

    // ret = encoder_position_read(&encoder_position_steps, encoder_position_init);
    // if (select_suspended_mode == 0) {
    //     encoder_position = encoder_position_steps - encoder_position_down - (int)(180 * angle_scale);
    //     encoder_position = encoder_position - encoder_position_offset;
    // }

    

}
bool InvertedPendulumGAM::control_logic_State_PendulumStablisation() {

    ret = encoder_position_read(&encoder_position_steps, encoder_position_init);

    /* Calibrate down angle */
    if( control_logic_State_PendulumStablisation_testCount == 2 ){
        control_logic_State_PendulumStablisation_testCount++;
         /*
            * Initialize Pendulum Angle Read offset by setting encoder_position_init
        */
        encoder_position_init = encoder_position_steps; 
        return false; 

    }else if( control_logic_State_PendulumStablisation_testCount == 3 ){
        encoder_position_down = encoder_position_steps;
        return true;
    }

    if( control_logic_State_PendulumStablisation_isSecondRead ){
        encoder_position_curr = encoder_position_steps;
        control_logic_State_PendulumStablisation_isSecondRead = false; 
    }else{
        encoder_position_prev = encoder_position_steps;
        control_logic_State_PendulumStablisation_isSecondRead = true;
    }

    if( (encoder_position_prev == encoder_position_curr)){  
        control_logic_State_PendulumStablisation_testCount++;   
        // if(control_logic_State_PendulumStablisation_testCount == 2)
        //     return true;
    }
 

    return false;
}

bool InvertedPendulumGAM::control_logic_State_SwingingUp() {

    
        *OUTPUT_rotor_control_target_steps= 0;
		/* Enter Swing Up Loop */
		//while (1)
		//{
			//break;
			//HAL_Delay(2);
			ret = encoder_position_read(&encoder_position_steps, encoder_position_init);
			/* Optional Swing Up progress reporting */
			//sprintf(tmp_string,"Rotor Impulse Amplitude %i Max Angle (degrees) %0.3f\r\n", stage_amp, fabs((float)(global_max_encoder_position)/(ENCODER_READ_ANGLE_SCALE)));
			//HAL_UART_Transmit(&huart2, (u_int8_t*) tmp_string, strlen(tmp_string), HAL_MAX_DELAY);

			/* Break if pendulum angle relative to vertical meets tolerance (for clockwise or counter clockwise approach */
			if (fabs(encoder_position_steps - encoder_position_down - (int) (180 * angle_scale)) < START_ANGLE * angle_scale){
				return true;//state change
			}
			if (fabs(encoder_position_steps - encoder_position_down + (int)(180 * angle_scale)) < START_ANGLE * angle_scale){
				encoder_position_down = encoder_position_down - 2*(int)(180 * angle_scale);
				return true;//state change
			}

			if (zero_crossed)
			{//
				zero_crossed = false;
				// Push it aka put some more kinetic energy into the pendulum
				if (swing_up_state == 0){
					//BSP_MotorControl_Move(0, swing_up_direction, stage_amp);
					//BSP_MotorControl_WaitWhileActive(0);
                    *OUTPUT_rotor_control_target_steps=stage_amp;
                    *OUTPUT_gpioState = swing_up_direction;
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
                    return false;
					//ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
				}
			}


			// We have a peak but did not handle it yet
			if (peaked && !handled_peak)
			{
				// Ensure we only enter this branch one per peak
				handled_peak = true;
				// Reset maximum encoder value to reassess after crossing the bottom
				max_encoder_position = 0;
				// Switch motor direction
				swing_up_direction = swing_up_direction == FORWARD ? BACKWARD : FORWARD;
			}
		//}
    return false;
}

bool InvertedPendulumGAM::control_logic_State_Main() {
    /*
        *
        * Restore user selected control parameters after completion of Swing Up and if Angle Calibration
        * not enabled.  Start up delay permits settling prior to switching to new control parameters
        *
        */

   //encoder_position  = *INPUT_encoder_position;
   
   //MARTe::uint32 L6474_Board_Pwm1Counter = *INPUT_L6474_Board_Pwm1Counter;
   //MARTe::uint32 CYCCNT                  = *INPUT_CYCCNT;
   rotor_position_steps    = *INPUT_rotor_position_steps;

//set the exit_control_loop falg
   *OUTPUT_break_Control_Loop = 0u;

    if (enable_swing_up == 1 && i == SWING_UP_CONTROL_CONFIG_DELAY && enable_angle_cal == 0){
        PID_Rotor.Kp = init_r_p_gain;
        PID_Rotor.Ki = init_r_i_gain;
        PID_Rotor.Kd = init_r_d_gain;
        PID_Pend.Kp = init_p_p_gain;
        PID_Pend.Ki = init_p_i_gain;
        PID_Pend.Kd = init_p_d_gain;
        enable_state_feedback = init_enable_state_feedback;
        integral_compensator_gain = init_integral_compensator_gain;
        feedforward_gain = init_feedforward_gain;
        enable_state_feedback = init_enable_state_feedback;
        enable_disturbance_rejection_step = init_enable_disturbance_rejection_step;
        enable_sensitivity_fnc_step = init_enable_sensitivity_fnc_step;
        enable_noise_rejection_step = init_enable_noise_rejection_step;
        enable_rotor_plant_design = init_enable_rotor_plant_design;
        enable_rotor_plant_gain_design = init_enable_rotor_plant_gain_design;
    }

    /*
        *  Real time user configuration and mode assignment
        */

    //mode_index_prev = mode_index;

    // RxBuffer_WriteIdx = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    // readBytes = Extract_Msg(RxBuffer, RxBuffer_ReadIdx, RxBuffer_WriteIdx, UART_RX_BUFFER_SIZE, &Msg);

    config_command = 0;
    /* Test for NULL message received corresponding to keyboard Carriage Return */
    // if (readBytes == 1){
    //     RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;
    //     continue;
    // }
    /* Test for message received */
    //##################TO REVISIT ##################################
    // if (readBytes == 2 && Msg.Len == 1 && i % 10 == 0){
    //     RxBuffer_ReadIdx = (RxBuffer_ReadIdx + readBytes) % UART_RX_BUFFER_SIZE;
    //     mode_transition_state = 1;
    //     /* Determine user input */
    //     mode_index_command = mode_index_identification((char *)Msg.Data, config_command, & adjust_increment,
    //             &PID_Pend, &PID_Rotor);
    //     strcpy(config_message, (char *) Msg.Data);
    //     if (strcmp(config_message, ">") == 0){
    //         // Logging was activated from the Real-Time Workbench
    //         if (enable_full_sysid && full_sysid_start_index == -1) {
    //             full_sysid_start_index = i + 50;
    //         }
    //     } else if (strcmp(config_message, "q") == 0){
    //         sprintf(msg_display,
    //                 "\n\rExit Control Loop Command Received ");
    //         HAL_UART_Transmit(&huart2, (u_int8_t*) msg_display,
    //                 strlen(msg_display), HAL_MAX_DELAY);
    //         break;
    //     }
    // }

    /* Set mode 1 if user request detected */
    // if (mode_index_command == 1 && mode_transition_state == 1) {
    //     mode_index = 1;
    //     mode_transition_state = 0;
    //     mode_index_command = 0;
    //     assign_mode_1(&PID_Pend, &PID_Rotor);
    // }
    // /* Set mode 3 if user request detected */
    // if (mode_index_command == 2 && mode_transition_state == 1) {
    //     mode_index = 2;
    //     mode_transition_state = 0;
    //     mode_index_command = 0;
    //     assign_mode_2(&PID_Pend, &PID_Rotor);
    // }
    // /* Set mode 3 if user request detected */
    // if (mode_index_command == 3 && mode_transition_state == 1) {
    //     mode_index = 3;
    //     mode_transition_state = 0;
    //     mode_index_command = 0;
    //     assign_mode_3(&PID_Pend, &PID_Rotor);
    // }
    /* End of Real time user configuration and mode assignment read loop */


    /* Exit control if cycle count limit set */
    //##################TO REVISIT ##################################
    // if (i > cycle_count && ENABLE_CYCLE_INFINITE == 0) {
    //     break;
    // }

    /*
        * *************************************************************************************************
        *
        * Initiate Measurement and Control
        *
        * *************************************************************************************************
        */

    /*
        * Optional Reset and clear integrator error during initial start of controllers
        */
    if (i < 1){
        PID_Pend.int_term = 0;
        PID_Rotor.int_term = 0;
    }

    /*
        * Acquire encoder position and correct for initial angle value of encoder measured at
        * vertical down position at system start including 180 degree offset corresponding to
        * vertical upwards orientation.
        *
        * For case of Suspended Mode Operation the 180 degree offset is not applied
        *
        * The encoder_position_offset variable value is determined by the Automatic Inclination
        * Angle Calibration system
        */

    //##################TO REVISIT ##################################
    ret = encoder_position_read(&encoder_position_steps, encoder_position_init);
    if (select_suspended_mode == 0) {
        encoder_position = encoder_position_steps - encoder_position_down - (int)(180 * angle_scale);
        encoder_position = encoder_position - encoder_position_offset;
    }
    if (select_suspended_mode == 1) {
        encoder_position = encoder_position_steps - encoder_position_down;
        encoder_position = encoder_position - encoder_position_offset;
    }

    /*  Detect pendulum position excursion exceeding limits and exit */

    if(select_suspended_mode == 0){
        if (((encoder_position)/ ENCODER_READ_ANGLE_SCALE) > ENCODER_POSITION_POSITIVE_LIMIT) {
            //sprintf(msg_display, "Error Exit Encoder Position Exceeded: %i\r\n", encoder_position_steps);
            *OUTPUT_break_Control_Loop = 1u;
            return false;
            //##################TO REVISIT QUIT HERE##################################
            //break;
        }
        if (((encoder_position)/ ENCODER_READ_ANGLE_SCALE) < ENCODER_POSITION_NEGATIVE_LIMIT) {
            //sprintf(msg_display, "Error Exit Encoder Position Exceeded: %i\r\n", encoder_position_steps);
            *OUTPUT_break_Control_Loop = 1u;
            return false;
            //##################TO REVISIT QUIT HERE##################################
            //break;
        }
    }

    // /* Detect rotor position excursion exceeding limits and exit */

    if (rotor_position_steps > (ROTOR_POSITION_POSITIVE_LIMIT * STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
        // sprintf(msg_display, "Error Exit Motor Position Exceeded: %i\r\n", rotor_position_steps);
        *OUTPUT_break_Control_Loop = 1u;
        return false;
        //##################TO REVISIT QUIT HERE##################################
        //break;
    }

    if (rotor_position_steps < (ROTOR_POSITION_NEGATIVE_LIMIT * STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
        // sprintf(msg_display, "Error Exit Motor Position Exceeded: %i\r\n", rotor_position_steps);
        *OUTPUT_break_Control_Loop = 1u;
        return false;
        //##################TO REVISIT QUIT HERE##################################
        //break;
    }

    /*
        * Encoder Angle Error Compensation
        *
        * Compute Proportional control of pendulum angle compensating for error due to
        * encoder offset at start time or system platform slope relative to horizontal.
        *
        * Apply optional time limit to correct for encoder error or slope.  For cycle count
        * greater than ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT, encoder angle
        * slope correction remains constant.
        *
        * If ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT = 0, then encoder angle slope
        * correction continues operation for all time
        *
        * Note: This system is *not* required in the event that Automatic Inclination
        * Angle Calibration is selected.
        *
        */

    /* Compute Low Pass Filtered rotor position difference */

    rotor_position_diff_prev = rotor_position_diff;

    if (enable_disturbance_rejection_step == 0){
        rotor_position_diff = rotor_position_filter_steps
                - rotor_position_command_steps;
    }
    if (enable_disturbance_rejection_step == 1){
        rotor_position_diff = rotor_position_filter_steps;
    }



    /* Apply slope correction */
    if (ENABLE_ENCODER_ANGLE_SLOPE_CORRECTION == 1 && i > angle_cal_complete) {
        rotor_position_diff_filter =
                (float) (rotor_position_diff * iir_LT_0)
                + rotor_position_diff_prev * iir_LT_1
                - rotor_position_diff_filter_prev * iir_LT_2;
        if ((i < ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT) || (ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT == 0)) {
            encoder_angle_slope_corr_steps = rotor_position_diff_filter / ENCODER_ANGLE_SLOPE_CORRECTION_SCALE;
        }
        rotor_position_diff_filter_prev = rotor_position_diff_filter;
    }


    /*
        *  Compute current_error_steps input for Primary Controller
        *
        *  current_error_steps is sum of encoder angle error compensation and encoder position
        *  in terms of stepper motor steps
        *
        *  An Encoder offset may be introduced.  The Encoder offset may remain at all times if
        *  ENCODER_OFFSET_DELAY == 0 or terminate at a time (in ticks) of ENCODER_OFFSET_DELAY
        */

    if ((i > ENCODER_START_OFFSET_DELAY) || (ENCODER_START_OFFSET_DELAY == 0)){
        encoder_position = encoder_position - ENCODER_START_OFFSET;
    }

    /*
        * Compute error between Pendulum Angle and Pendulum Tracking Angle in units of steps
        * Apply scale factor to match angle to step gain of rotor actuator
        *
        */

    *current_error_steps = encoder_angle_slope_corr_steps
    + ENCODER_ANGLE_POLARITY * (encoder_position / ((float)(ENCODER_READ_ANGLE_SCALE/STEPPER_READ_POSITION_STEPS_PER_DEGREE)));

    /*
        *
        * Pendulum Controller execution
        *
        * Include addition of Pendulum Angle track signal impulse signal
        * Compute control signal, rotor position target in step units
        *
        * Pendulum tracking command, pendulum_position_command, also supplied in step units
        *
        */

    *current_error_steps = *current_error_steps + pendulum_position_command_steps;

    pid_filter_control_execute(&PID_Pend,current_error_steps, sample_period, Deriv_Filt_Pend);

    rotor_control_target_steps = PID_Pend.control_output;

    /* Acquire rotor position and compute low pass filtered rotor position */


   //##################TO REVISIT ##################################
    //ret = rotor_position_read(&rotor_position_steps);
    
    /* Optional rotor position filter */

    rotor_position_filter_steps = (float) (rotor_position_steps) * iir_0 + rotor_position_steps_prev * iir_1
            - rotor_position_filter_steps_prev * iir_2;
    rotor_position_steps_prev = (float) (rotor_position_steps);
    rotor_position_filter_steps_prev = rotor_position_filter_steps;


    rotor_position_filter_steps = rotor_position_steps;

    /*
        * 		Compute rotor chirp tracking signal with chirp sweep from rotor_chirp_start_freq
        * 		to rotor_chirp_end_freq in time period rotor_chirp_period in units of control
        * 		loop cycle periods.  Each chirp is separated by delay of ROTOR_CHIRP_SWEEP_DELAY.
        */

    if (enable_rotor_chirp == 1 && enable_mod_sin_rotor_tracking == 0
            && enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete) {

        if (i < ROTOR_CHIRP_PERIOD - 1){
            chirp_cycle = 0;
        }
        if (chirp_cycle > ROTOR_CHIRP_PERIOD - 1) {
            chirp_cycle = 0;
            chirp_dwell_cycle = ROTOR_CHIRP_SWEEP_DELAY;
        }
        if (chirp_dwell_cycle > 0){
            chirp_dwell_cycle--;
            chirp_cycle = 0;
        }
        if (chirp_dwell_cycle == 0 && i >= ROTOR_CHIRP_PERIOD - 1){
            chirp_cycle = chirp_cycle + 1;
            chirp_time = (float)((chirp_cycle - 1)/ROTOR_CHIRP_SAMPLE_RATE);
            rotor_chirp_frequency = rotor_chirp_start_freq + (rotor_chirp_end_freq - rotor_chirp_start_freq)*((float)(chirp_cycle/rotor_chirp_period));
            rotor_position_command_steps = ((float)(ROTOR_CHIRP_STEP_AMPLITUDE*STEPPER_READ_POSITION_STEPS_PER_DEGREE))*sin(2.0*3.14159*rotor_chirp_frequency*chirp_time);
        }
    }


    /*  Create rotor track "comb" signal */
    if (enable_rotor_tracking_comb_signal > 0 && i > 1000 && i > angle_cal_complete) {

        chirp_time = ((float)(i - 1))/500.0;
        rotor_track_comb_signal_frequency = 0.01;
        rotor_track_comb_command = ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.017783;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.031623;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.056234;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.1;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.17783;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.31623;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 0.56234;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 1.0;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 1.7783;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 3.1623;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 5.6234;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
        rotor_track_comb_signal_frequency = 10;
        rotor_track_comb_command = rotor_track_comb_command + ((float)(rotor_track_comb_amplitude))*sin(rotor_track_comb_signal_frequency*chirp_time);
    }

    if (enable_rotor_chirp == 0 && enable_mod_sin_rotor_tracking == 0
            && enable_rotor_tracking_comb_signal == 1) {
        rotor_position_command_steps = rotor_track_comb_command;
    }

    /*  Create rotor angle reference tracking modulated sine signal */

    rotor_sine_drive = 0;
    if (enable_mod_sin_rotor_tracking == 1 && ENABLE_ROTOR_CHIRP == 0 && i > angle_cal_complete) {

        if (ENABLE_ROTOR_CHIRP == 0){
            mod_sin_carrier_frequency = MOD_SIN_CARRIER_FREQ;
        }

        if (i > MOD_SIN_START_CYCLES && enable_mod_sin_rotor_tracking == 1) {
            rotor_sine_drive =
                    (float) (mod_sin_amplitude
                            * (1 + sin(-1.5707 + ((i - MOD_SIN_START_CYCLES)/MOD_SIN_SAMPLE_RATE) * (MOD_SIN_MODULATION_FREQ * 6.2832))));
            rotor_sine_drive_mod = sin(0 + ((i - MOD_SIN_START_CYCLES) /MOD_SIN_SAMPLE_RATE) * (mod_sin_carrier_frequency * 6.2832));
            rotor_sine_drive = rotor_sine_drive + MOD_SIN_MODULATION_MIN;
            rotor_sine_drive = rotor_sine_drive * rotor_sine_drive_mod * rotor_mod_control;
        }

        if (i > MOD_SIN_START_CYCLES && ENABLE_SIN_MOD == 0) {
            rotor_sine_drive_mod = sin(0 + ((i - MOD_SIN_START_CYCLES) /MOD_SIN_SAMPLE_RATE) * (mod_sin_carrier_frequency * 6.2832));
            rotor_sine_drive = rotor_control_sin_amplitude * rotor_sine_drive_mod * rotor_mod_control;
        }

        if ( fabs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 1 && sine_drive_transition == 1){
            rotor_mod_control = 0.0;
            sine_drive_transition = 0;
        }
        if ( fabs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 0 && sine_drive_transition == 1){
            rotor_mod_control = 1.0;
            sine_drive_transition = 0;
        }

        if (enable_rotor_position_step_response_cycle == 0){
            rotor_position_command_steps = rotor_sine_drive;
        }

    }

    /*  Create rotor angle reference tracking impulse signal */

    if (ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE == 1 && i != 0 && i > angle_cal_complete) {
        if ((i % ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
            rotor_position_command_steps =
                    (float) (ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
                            * STEPPER_READ_POSITION_STEPS_PER_DEGREE);
            impulse_start_index = 0;
        }
        if (impulse_start_index
                > ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
            rotor_position_command_steps = 0;
        }
        impulse_start_index++;
    }

    /*
        * Create pendulum angle reference tracking impulse signal.  Polarity of impulse alternates
        */

    if (enable_pendulum_position_impulse_response_cycle == 1 && i != 0 && i > angle_cal_complete) {

        if ((i % PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
            if (select_suspended_mode == 1) {
                pendulum_position_command_steps =
                        (float) PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE;
            }
            if (select_suspended_mode == 0) {
                pendulum_position_command_steps =
                        (float) (PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
                                /PENDULUM_POSITION_IMPULSE_AMPLITUDE_SCALE);
            }
            chirp_cycle = 0;
            impulse_start_index = 0;
        }
        if (impulse_start_index
                > PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
            pendulum_position_command_steps = 0;
        }
        impulse_start_index++;
        chirp_cycle++;
    }

    /*  Create rotor angle reference tracking  step signal */

    if ((i % ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL) == 0 && enable_rotor_position_step_response_cycle == 1 && i > angle_cal_complete) {
        rotor_position_step_polarity = -rotor_position_step_polarity;
        if (rotor_position_step_polarity == 1){
            chirp_cycle = 0;
        }
    }

    if (enable_rotor_position_step_response_cycle == 1 && enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete) {
        if (STEP_RESPONSE_AMP_LIMIT_ENABLE == 1 && abs(rotor_sine_drive) > STEP_RESPONSE_AMP_LIMIT){
            chirp_cycle = chirp_cycle + 1;
        } else {
            if (enable_mod_sin_rotor_tracking == 1){
                rotor_position_command_steps = rotor_sine_drive + (float) ((rotor_position_step_polarity)
                        * ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
                        * STEPPER_READ_POSITION_STEPS_PER_DEGREE);
            }
            if (enable_mod_sin_rotor_tracking == 0){
                rotor_position_command_steps_pf = (float) ((rotor_position_step_polarity)
                        * ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
                        * STEPPER_READ_POSITION_STEPS_PER_DEGREE);
            }
            chirp_cycle = chirp_cycle + 1;
        }
    }

    /*
        * Rotor tracking reference, rotor_position_command_steps, is low pass filtered to prevent
        * aliasing of measurement during operation of Real Time Workbench sampling that occurs at
        * 50 Hz (in support of connected computing platform bandwidth limitations).  This filter
        * application is not applied during selection of high speed sampling at 500 Hz.
        */

    if (enable_rotor_position_step_response_cycle == 1 && enable_mod_sin_rotor_tracking == 0 && enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete){
        rotor_position_command_steps = rotor_position_command_steps_pf * iir_0_s
                + rotor_position_command_steps_pf_prev * iir_1_s
                - rotor_position_command_steps_prev * iir_2_s;
        rotor_position_command_steps_pf_prev = rotor_position_command_steps_pf;
    }

    /*
        *  Automatic Inclination Angle Calibration System
        *
        *  The Edukit system may be resting on a surface with a slight incline. This then
        *  produces a Rotor Angle dependent error between the measurement of Pendulum Angle
        *  and the angle corresponding to true vertical of the gravitational vector.
        *
        *  This system computed true vertical angle relative to the gravity vector for each
        *  Rotor Step.  This provides an encoder_offset_angle calibration value for all
        *  orientations of the Rotor.
        */

    if (enable_angle_cal == 1){
        /*
            * Angle Calibration system state values applied during Angle Calibration
            * Period.  User selected system state values restored after Angle Calibration
            */

        if (i == 1 && select_suspended_mode == 0){
            PID_Rotor.Kp = 21.1;
            PID_Rotor.Ki = 0;
            PID_Rotor.Kd = 17.2;
            PID_Pend.Kp = 419;
            PID_Pend.Ki = 0.0;
            PID_Pend.Kd = 56;
            enable_state_feedback = 1;
            integral_compensator_gain = 10;
            feedforward_gain = 1;
            rotor_position_command_steps = 0;
            current_error_rotor_integral = 0;
        }

        if (i == 1 && select_suspended_mode == 1){
            PID_Rotor.Kp = -23.86;
            PID_Rotor.Ki = 0;
            PID_Rotor.Kd = -19.2;
            PID_Pend.Kp = -293.2;
            PID_Pend.Ki = 0.0;
            PID_Pend.Kd = -41.4;
            enable_state_feedback = 1;
            integral_compensator_gain = -11.45;
            feedforward_gain = 1;
            rotor_position_command_steps = 0;
            current_error_rotor_integral = 0;
        }


        /* Initialize angle calibration variables */

        if (i == 1){
            offset_end_state = 0;
            offset_start_index = 4000;					// initial start index for sweep
            angle_index = ANGLE_CAL_OFFSET_STEP_COUNT;	// Number of angle steps
            angle_cal_end = INT_MAX;
            angle_cal_complete = INT_MAX;				// Allowed start time for stimulus signals
            encoder_position_offset_zero = 0;
        }

        if (offset_end_state == 0){
            /* Suspend loop delay warning since computation may lead to control loop cycle delay during
                * period after measurement and during computation of smoothed offset data array
                */
            enable_cycle_delay_warning = 0;
            /* Advance to upper angle of 90 degrees*/
            if (i > 1 && i < 4000){
                rotor_position_command_steps = (i/4000.0) * ANGLE_CAL_OFFSET_STEP_COUNT/2;
                offset_start_index = i + 4000;
            }
            /* Delay for time increment to avoid transient response in measurement.
                * Acquire samples for time-average of offset
                */
            if (i >= offset_start_index && i < (offset_start_index + 10)){
                //offset_angle[angle_index] = offset_angle[angle_index] + encoder_position;
                //offset_angle[angle_index] = encoder_position;
            }
            /* Compute time-averages offset and advance to next lower angle increment */
            if (i == offset_start_index + 10 && angle_index > 0){
                offset_angle[angle_index] = encoder_position;
                //offset_angle[angle_index] = offset_angle[angle_index]/10;
                angle_index = angle_index - 1;
                offset_start_index = offset_start_index + 10;
                rotor_position_command_steps = rotor_position_command_steps - 1;
            }

            /* Compute average encoder position offset over angle index range from ANGLE_AVG_SPAN to ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN */
            /* Suspend delay warning */

            if (angle_index >= 2*ANGLE_AVG_SPAN && angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1){
                for (angle_avg_index = angle_index - 2*ANGLE_AVG_SPAN; angle_avg_index < (angle_index + 1); angle_avg_index++){
                    encoder_position_offset_avg[angle_index] = 0;
                            for (angle_avg_index = angle_index - ANGLE_AVG_SPAN; angle_avg_index < (1 + angle_index + ANGLE_AVG_SPAN); angle_avg_index++){
                                    encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index] + offset_angle[angle_avg_index];
                            }
                            encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index]/(float)(2*ANGLE_AVG_SPAN + 1);
                }
            }

            /* Restore rotor angle to zero degrees */
            if (angle_index == 0){
                rotor_position_command_steps = rotor_position_command_steps + 0.02*STEPPER_READ_POSITION_STEPS_PER_DEGREE;
            }
            /* Terminate offset measurement and initialize angle_cal_end at time of termination */
            if (rotor_position_command_steps >= 0 && angle_index == 0){
                offset_end_state = 1;
                angle_cal_end = i;
                /* Restore loop delay warning */
                enable_cycle_delay_warning = 1;
            }
        }
    }

    /* Apply offset angle for correction of pendulum angle according to rotor position */

    if (offset_end_state == 1 && i > angle_cal_end){
        /* Compute angle index corresponding to rotor position */
        angle_index = (int)((ANGLE_CAL_OFFSET_STEP_COUNT - 1)/2) + rotor_position_filter_steps;
        if (angle_index < ANGLE_AVG_SPAN ){
            angle_index = ANGLE_AVG_SPAN;
        }
        if (angle_index >  ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN ){
            angle_index =  ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN;
        }
        encoder_position_offset = 2.0 * encoder_position_offset_avg[angle_index];
    }

    /* Measure residual offset at zero rotor position */
    if (offset_end_state == 1 && i > angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING && i < angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL){
        encoder_position_offset_zero = encoder_position_offset_zero + encoder_position;
    }

    /* Correct offset angle array values for any residual offset */
    if (i == (angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL + 1)){
        encoder_position_offset_zero = encoder_position_offset_zero/ANGLE_CAL_ZERO_OFFSET_DWELL;
        for (angle_index = 0; angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1; angle_index++){
            encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index] + encoder_position_offset_zero;
        }
        angle_cal_complete = angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL + 1 + ANGLE_CAL_COMPLETION;
    }

    /* Restore user selected system state configuration */
    if (offset_end_state == 1 && (enable_angle_cal == 1) && i == angle_cal_complete + 1){
        PID_Rotor.Kp = init_r_p_gain;
        PID_Rotor.Ki = init_r_i_gain;
        PID_Rotor.Kd = init_r_d_gain;
        PID_Pend.Kp = init_p_p_gain;
        PID_Pend.Ki = init_p_i_gain;
        PID_Pend.Kd = init_p_d_gain;
        current_error_rotor_integral = 0;
        enable_state_feedback = init_enable_state_feedback;
        integral_compensator_gain = init_integral_compensator_gain;
        feedforward_gain = init_feedforward_gain;
        enable_state_feedback = init_enable_state_feedback;
        enable_disturbance_rejection_step = init_enable_disturbance_rejection_step;
        enable_sensitivity_fnc_step = init_enable_sensitivity_fnc_step;
        enable_noise_rejection_step = init_enable_noise_rejection_step;
        enable_rotor_plant_design = init_enable_rotor_plant_design;
    }


    if (ENABLE_DUAL_PID == 1) {

        /*
            * Secondary Controller execution including Sensitivity Function computation
            */

        if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 0){
            *current_error_rotor_steps = rotor_position_filter_steps - rotor_position_command_steps;
        }
        if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 1 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 0){
            *current_error_rotor_steps = rotor_position_filter_steps;
        }
        if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 1){
            *current_error_rotor_steps = rotor_position_filter_steps + rotor_position_command_steps;
        }

        if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 1 && enable_noise_rejection_step == 0){
            *current_error_rotor_steps = rotor_position_filter_steps - rotor_position_command_steps;
        }

        /*
            * Select Reference signal input location at input of controller for Dual PID architecture
            * for Output Feedback Architecture or at output of controller and plant input for Full State
            * Feedback Architecture
            */

        if (enable_state_feedback == 1){
            *current_error_rotor_steps = rotor_position_filter_steps;
        }

        /*
            * PID input supplied in units of stepper motor steps with tracking error determined
            * as difference between rotor position tracking command and rotor position in units
            * of stepper motor steps.
            */

        pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps,
                sample_period_rotor,  Deriv_Filt_Rotor);

        rotor_control_target_steps = PID_Pend.control_output + PID_Rotor.control_output;


        if (enable_state_feedback == 1 && integral_compensator_gain != 0){
            /*
                * If integral action is included, state feedback plant input equals
                * the time integral of difference between reference tracking signal and rotor angle,
                * current_error_rotor_integral.  This is summed with the controller output, rotor_control_target_steps.
                * The integral_compensator_gain as input by user includes multiplicative scale factor matching
                * scaling of controller gain values.
                */
            current_error_rotor_integral = current_error_rotor_integral + (rotor_position_command_steps*feedforward_gain - rotor_position_filter_steps)*(*sample_period_rotor);
            rotor_control_target_steps = rotor_control_target_steps - integral_compensator_gain*current_error_rotor_integral;
        }

        if (enable_state_feedback == 1 && integral_compensator_gain == 0){
            /*
                * If integral compensator is not included, full state feedback plant input equals difference
                * between control output and reference tracking signal with scale factor matching
                * scaling of controller gain values.
                *
                * If Plant Design system is applied, the effects of small numerical error in transfer function
                * computation is compensated for by removal of average offset error.
                *
                */

            rotor_control_target_steps = rotor_control_target_steps - rotor_position_command_steps*feedforward_gain;

        }


        /*
            * Load Disturbance Sensitivity Function signal introduction with scale factor applied to increase
            * amplitude of Load Disturbance signal to enhance signal to noise in measurement.  This scale factor
            * then must be applied after data acquisition to compute proper Load Disturbance Sensitivity Function.
            * Note that Load Disturbance Sensitivity Function value is typically less than -20 dB
            *
            */
        if (enable_disturbance_rejection_step == 1){
            rotor_control_target_steps = rotor_control_target_steps + rotor_position_command_steps * load_disturbance_sensitivity_scale;
        }
    }


    if (full_sysid_start_index != -1 && i >= full_sysid_start_index && i > angle_cal_complete) {
        float total_acc = 0;
        float t = (i - full_sysid_start_index) * Tsample;
        float w = full_sysid_min_freq_hz * M_TWOPI;
        for (int k_step = 0; k_step < full_sysid_num_freqs; k_step++) {
            float wave_value = w * cosf(w * t); // multiply acceleration wave by omega to keep consistent velocity amplitude
            total_acc += wave_value;
            w *= full_sysid_freq_log_step;
        }
        rotor_control_target_steps = ((full_sysid_max_vel_amplitude_deg_per_s/full_sysid_num_freqs) * total_acc * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE);
    }


    /*
        *
        * Plant transfer function design based on two stage first order high pass IIR
        * filter structures applied to rotor_control_target_steps.
        *
        * Second order system computed at all cycle times to avoid transient upon switching between
        * operating modes with and without Rotor Plant Design enabled
        *
        */


    if (rotor_damping_coefficient != 0 || rotor_natural_frequency != 0){

            rotor_control_target_steps_filter_2 = c0* rotor_control_target_steps + c1* rotor_control_target_steps_prev
                    + c2*rotor_control_target_steps_prev_prev + c3*rotor_control_target_steps_filter_prev_2
                    + c4*rotor_control_target_steps_filter_prev_prev_2;

            rotor_control_target_steps_prev_prev = rotor_control_target_steps_prev;
            rotor_control_target_steps_filter_prev_prev_2 = rotor_control_target_steps_filter_prev_2;
            rotor_control_target_steps_filter_prev_2 = rotor_control_target_steps_filter_2;
    }

    if ((enable_rotor_plant_design == 2 )){
        rotor_control_target_steps_filter_2 = iir_0_r* rotor_control_target_steps + iir_1_r*rotor_control_target_steps_prev
                - iir_2_r*rotor_control_target_steps_filter_prev_2;
        rotor_control_target_steps_filter_prev_2 = rotor_control_target_steps_filter_2;
    }


    /*
        * Record current value of rotor_position_command tracking signal
        * and control signal, rotor_control_target_steps for rotor position
        * rotor position filters, rotor plant design, performance monitoring and adaptive control
        */

    rotor_control_target_steps_prev = rotor_control_target_steps;
    rotor_position_command_steps_prev = rotor_position_command_steps;

    //##################TO REVISIT HARWARE CALL TO TAKE ACTION##################################
    // if (ACCEL_CONTROL == 1) {
    //     if (enable_rotor_plant_design != 0){
    //         rotor_control_target_steps_filter_2 = rotor_plant_gain*rotor_control_target_steps_filter_2;
    //         apply_acceleration(&rotor_control_target_steps_filter_2, &target_velocity_prescaled, Tsample);
    //     /* Applies if Rotor Gain defined */
    //     } else if (enable_rotor_plant_gain_design == 1){
    //         rotor_control_target_steps_gain = rotor_plant_gain * rotor_control_target_steps;
    //         apply_acceleration(&rotor_control_target_steps_gain, &target_velocity_prescaled, Tsample);
    //     /* Applies if no Rotor Design is selected */
    //     } else {
    //         apply_acceleration(&rotor_control_target_steps, &target_velocity_prescaled, Tsample);
    //     }
    //     *OUTPUT_rotor_control_target_steps = 0;
    // } else {
    //     *OUTPUT_rotor_control_target_steps = rotor_control_target_steps/2;
    //     //BSP_MotorControl_GoTo(0, rotor_control_target_steps/2);
    // }
    *OUTPUT_rotor_control_target_steps = rotor_control_target_steps;
    /*
        * *************************************************************************************************
        *
        * Data Report Sequence Start
        *
        * *************************************************************************************************
        */


    if (enable_pendulum_position_impulse_response_cycle == 1) {
        reference_tracking_command = pendulum_position_command_steps;
    } else {
        reference_tracking_command = rotor_position_command_steps;
    }

    //##################TO REVISIT HARWARE SET-UP control cycle heartbit ##################################
    // /* Compute 100 cycle time average of cycle period for system performance measurement */
    // if(i == 1){
    //     cycle_period_start = HAL_GetTick();
    //     cycle_period_sum = 100*Tsample*1000 - 1;
    // }
    // if(i % 100 == 0){
    //     cycle_period_sum = HAL_GetTick() - cycle_period_start;
    //     cycle_period_start = HAL_GetTick();
    // }
    // tick = HAL_GetTick();
    // tick_cycle_previous = tick_cycle_current;
    // tick_cycle_current = tick;



    /* High speed sampling mode data reporting */
    /* Time reported is equal to ((cycle time - 2000)microseconds/10) */

    // if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 1 && enable_rotor_tracking_comb_signal == 0 && ACCEL_CONTROL_DATA == 0){
    //     sprintf(msg_display, "%i\t%i\t%i\t%i\t%i\r\n", cycle_period_sum - 200, (int)(roundf(encoder_position)), display_parameter,
    //             (int)(roundf(rotor_control_target_steps)),(int)(reference_tracking_command));
    //     show_error();
    // }

    /* High speed sampling mode data reporting without rotor chirp signal and with comb signal */
    // if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0 && enable_rotor_tracking_comb_signal == 1 && ACCEL_CONTROL_DATA == 0){
    //     sprintf(msg_display, "%i\t%i\t%i\t%i\t%i\r\n", current_cpu_cycle_delay_relative_report,
    //             (int)(roundf(encoder_position)), display_parameter, (int)(roundf(rotor_control_target_steps)), (int)(roundf(100*rotor_position_command_steps)));
    //     show_error();
    // }

    /* High speed sampling mode data reporting without rotor chirp signal and without comb signal */
    /* Time reported is equal to ((cycle time - sample time)microseconds/10) */
    // if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0 && enable_rotor_tracking_comb_signal == 0 && ACCEL_CONTROL_DATA == 0){

    //     sprintf(msg_display, "%i\t%i\t%i\t%i\t%i\r\n", cycle_period_sum - 200,
    //             (int)(roundf(encoder_position)), display_parameter,
    //             (int)(roundf(rotor_control_target_steps)),(int)(reference_tracking_command));
    //     show_error();
    // }

    if (enable_high_speed_sampling == 1 && enable_rotor_chirp == 0 && ACCEL_CONTROL_DATA == 1){
        if (enable_pendulum_position_impulse_response_cycle == 1) {
            reference_tracking_command = pendulum_position_command_steps;
        } else {
            reference_tracking_command = rotor_position_command_steps;
        }

        // if (Tsample <= 0.00125) { // 1/800Hz = 0.00125s
        //     /* High speed sampling mode data reporting for 800 Hz mode */
        //     sprintf(msg_display, "%i\t%i\r\n", (int)reference_tracking_command, current_pwm_period);
        //     show_error();
        // } else {
        //     /* High speed sampling mode data reporting for 500 Hz mode with acceleration contol system data */
        //     sprintf(msg_display, "%i\t%i\t%i\t%i\t%i\t%i\r\n",
        //             (int)reference_tracking_command, (int)(roundf(rotor_control_target_steps/10)),(int)(rotor_position_command_steps),
        //             current_pwm_period, desired_pwm_period/10000,
        //             (clock_int_time/100000));
        //     show_error();
        // }
    }

    /* Select display parameter corresponding to requested selection of Sensitivity Functions */
    if (enable_disturbance_rejection_step == 1) { display_parameter = rotor_position_steps/load_disturbance_sensitivity_scale; }
    else if (enable_noise_rejection_step == 1) { noise_rej_signal = rotor_control_target_steps; }
    else if (enable_sensitivity_fnc_step == 1)  { display_parameter = rotor_position_command_steps - rotor_position_steps; }
    else { display_parameter = rotor_position_steps; }


    if (enable_noise_rejection_step == 1){
        display_parameter = noise_rej_signal;
    }

    /*
        * Normal mode data reporting provides data output each 10th cycle
        * Time reported is the average of ((cycle periods - desired sample period) microseconds)/10
        */

    if (enable_high_speed_sampling == 0){

        /*
            * Provide report each 10th control cycle
            * Control parameters scaled by 100 to reduce communication bandwidth
            *
            * Speed scale may be enabled to reduce communication bandwidth
            *
            */


        // if (report_mode != 1000 && report_mode != 2000 && speed_governor == 0){
        //     sprintf(msg_display, "%i\t%i\t%i\t%i\t%i\t%i\t%.1f\t%i\t%i\r\n", (int)2, cycle_period_sum - 200,
        //             current_cpu_cycle_delay_relative_report,
        //             (int)(roundf(encoder_position)), display_parameter, (int)(PID_Pend.int_term)/100,
        //             reference_tracking_command, (int)(roundf(rotor_control_target_steps)),
        //             (int)(PID_Rotor.control_output)/100);
        //     show_error();
        // }

        // if (report_mode != 1000 && report_mode != 2000 && (i % speed_scale) == 0 && speed_governor == 1){
        //     sprintf(msg_display, "%i\t%i\t%i\t%i\t%i\t%i\t%.1f\t%i\t%i\r\n", (int)2, cycle_period_sum - 200,
        //             current_cpu_cycle_delay_relative_report,
        //             (int)(roundf(encoder_position)), display_parameter, (int)(PID_Pend.int_term)/100,
        //             reference_tracking_command, (int)(roundf(rotor_control_target_steps)),
        //             (int)(PID_Rotor.control_output)/100);
        //     show_error();
        // }

        /* Provide reports each 1000th cycle of system parameters
            * Speed parameters scaled by 10 to reduce communication bandwidth
            */

        // if (report_mode == 1000){
        //     sprintf(msg_display, "%i\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%i\t%i\r\n", (int)0,
        //             PID_Pend.Kp, PID_Pend.Ki, PID_Pend.Kd,
        //             PID_Rotor.Kp, PID_Rotor.Ki, PID_Rotor.Kd,
        //             max_speed/10, min_speed/10);
        //     show_error();
        // }
        // if (report_mode == 2000){
        //     sprintf(msg_display, "%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\r\n", (int)1,
        //             (int)torq_current_val, max_accel, max_decel, enable_disturbance_rejection_step,
        //             enable_noise_rejection_step, enable_rotor_position_step_response_cycle,
        //             (int)(adjust_increment*10), enable_sensitivity_fnc_step);
        //     report_mode = 0;
        //     show_error();
        // }
        report_mode = report_mode + 1;
        
    }


    // /*
    //     * Adjust cycle delay to match t_sample_cpu_cycles.
    //     */

    // prev_target_cpu_cycle = target_cpu_cycle;
    // target_cpu_cycle += t_sample_cpu_cycles; // Increment target_cpu_cycle by sample time

    // current_cpu_cycle = DWT->CYCCNT;

    // // If there is time left until target_cpu_cycle
    // if (((int) (target_cpu_cycle - current_cpu_cycle)) > 0) {

    //     // If DWT->CYCCNT needs to overflow first
    //     if (current_cpu_cycle > target_cpu_cycle) {
    //         // Wait for DWT->CYCCNT to overflow
    //         do {
    //             last_cpu_cycle = current_cpu_cycle;
    //             current_cpu_cycle = DWT->CYCCNT;
    //         } while (current_cpu_cycle >= last_cpu_cycle);
    //     }

    //     DWT_Delay_until_cycle(target_cpu_cycle);
    // } else {
    //     // Provide warning and exit if delay exceeds 5 cycles
    //     if (current_cpu_cycle - target_cpu_cycle > t_sample_cpu_cycles*5 && enable_cycle_delay_warning == 1) {
    //         sprintf(msg_display, "Error: control loop lag\r\n");
    //         show_error();
    //         //##################TO REVISIT QUIT HERE##################################
    //         //break;
    //     }
    // }

    // /* Record current cpu cycle for delay computation at the end of loop execution */
    // current_cpu_cycle = DWT->CYCCNT;

    // /* Compute value of relative delay after insertion of delay adjust */
    // current_cpu_cycle_delay_relative_report = (int)(t_sample_cpu_cycles - (current_cpu_cycle - prev_cpu_cycle));
    // current_cpu_cycle_delay_relative_report = (current_cpu_cycle_delay_relative_report*1000000)/RCC_HCLK_FREQ;

    // prev_cpu_cycle = current_cpu_cycle;

    /* Increment cycle counter */

    i++;

    return true;
		
}


float InvertedPendulumGAM::L6474_Board_Pwm1PrescaleFreq( float freq ){

    const int BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER = 2; // value : 2
    const int TIMER_PRESCALER = 1024; //value: 1024

    return TIMER_PRESCALER * BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER * freq;
}

void InvertedPendulumGAM::apply_acceleration(float * acc, float * target_velocity_prescaled, float t_sample) {
	/*
	 *  Stepper motor acceleration, speed, direction and position control developed by Ryan Nemiroff
	 */

	u_int32_t current_pwm_period_local = current_pwm_period;
	u_int32_t desired_pwm_period_local = desired_pwm_period;

	/*
	 * Add time reporting
	 */

	//apply_acc_start_time = *INPUT_CYCCNT;

     
	MOTOR_DIRECTION old_dir = *target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

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
	MOTOR_DIRECTION new_dir = *target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

	if (*target_velocity_prescaled > L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED)) {
		*target_velocity_prescaled = L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED);
	} else if (*target_velocity_prescaled < -L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED)) {
		*target_velocity_prescaled = -L6474_Board_Pwm1PrescaleFreq(MAXIMUM_SPEED);
	}

	float speed_prescaled;
	if (new_dir == FORWARD ) {
		speed_prescaled = *target_velocity_prescaled;
	} else {
		speed_prescaled = *target_velocity_prescaled * -1;
		if (speed_prescaled == 0) speed_prescaled = 0; // convert negative 0 to positive 0
	}


	u_int32_t effective_pwm_period = desired_pwm_period_local;

	float desired_pwm_period_float = roundf(RCC_SYS_CLOCK_FREQ / speed_prescaled);
	if (!(desired_pwm_period_float < 4294967296.0f)) {
		desired_pwm_period_local = UINT_MAX;
	}else if( desired_pwm_period_float == 0){
        desired_pwm_period_local  = 1;
    } else {
		desired_pwm_period_local = (u_int32_t)(desired_pwm_period_float);
	}

    //******************set OUTPUT
	if (old_dir != new_dir) {
		*OUTPUT_gpioState = new_dir; 
	}else
        *OUTPUT_gpioState = ((u_int8_t)0xFF) ;

    *OUTPUT_L6474_Board_Pwm1Period =0u;
	if (current_pwm_period_local != 0) {
		u_int32_t pwm_count = *INPUT_L6474_Board_Pwm1Counter;
		u_int32_t pwm_time_left = current_pwm_period_local - pwm_count;
		if (pwm_time_left > PWM_COUNT_SAFETY_MARGIN) {
			if (old_dir != new_dir) {
				// pwm_time_left = effective_pwm_period - pwm_time_left; // One method for assignment of PWM period during switching directions. This has the effect of additional discrete step noise.
				pwm_time_left = effective_pwm_period; // Second method for assignment of PWM period during switching directions. This shows reduced discrete step noise.
			}

			u_int32_t new_pwm_time_left = ((u_int64_t) pwm_time_left * desired_pwm_period_local) / effective_pwm_period;
			if (new_pwm_time_left != pwm_time_left) {
				if (new_pwm_time_left < PWM_COUNT_SAFETY_MARGIN) {
					new_pwm_time_left = PWM_COUNT_SAFETY_MARGIN;
				}
				current_pwm_period_local = pwm_count + new_pwm_time_left;
				if (current_pwm_period_local < pwm_count) {
					current_pwm_period_local = UINT_MAX;
				}

				*OUTPUT_L6474_Board_Pwm1Period = current_pwm_period_local;
				current_pwm_period = current_pwm_period_local;
			}
		}
	} else {
		*OUTPUT_L6474_Board_Pwm1Period = desired_pwm_period_local;
		current_pwm_period = desired_pwm_period_local;
	}

	desired_pwm_period = desired_pwm_period_local;

}

bool InvertedPendulumGAM::Initialise(MARTe::StructuredDataI & data) {
    bool ok = GAM::Initialise(data);

    control_logic_Initialise();
    //control_logic_Initialise_interactive();
    // if (ok) {
    //     ok = data.Read("InputDataRate", input_data_rate);
    //     if (!ok) {
    //         REPORT_ERROR(ErrorManagement::ParametersError, "No input data rate has been specified");
    //     }
    // }
   
    
    return ok;
}

bool InvertedPendulumGAM::Setup() {
    StreamString gam_name;
    
    bool ok = GetQualifiedName(gam_name);
    if (!ok) {
        REPORT_ERROR(ErrorManagement::ParametersError, "Cannot get the qualified name");
    }

    if (ok) {
        uint32 nOfInputSignals = GetNumberOfInputSignals();
        ok = (nOfInputSignals == 5u); // Will need to be changed if any input signals are added or removed
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "%s::Number of input signals must be 5", gam_name.Buffer());
        }
    } 
    uint32 signalIdx;
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "MessageCount", InputSignals, UnsignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            INPUT_message_count = (uint32*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal message_count");
        }
    }
    // if (ok) {    
    //     ok = GAMCheckSignalProperties(*this, "encoder_position", InputSignals, Float32Bit, 0u, 1u, signalIdx);
    //     if (ok) {
    //         INPUT_encoder_position = (float32*) GetInputSignalMemory(signalIdx);
    //     } else {
    //         REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal encoder_position");
    //     }
    // }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "rotor_position_steps", InputSignals, SignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            INPUT_rotor_position_steps = (int32*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal rotor_position_steps ");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "L6474_Board_Pwm1Counter", InputSignals, UnsignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            INPUT_L6474_Board_Pwm1Counter = (uint32*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal L6474_Board_Pwm1Counter ");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "INPUT_break_Control_Loop", InputSignals, UnsignedInteger8Bit, 0u, 1u, signalIdx);
        if (ok) {
            INPUT_break_Control_Loop = (uint8*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal INPUT_break_Control_Loop");
        }
    }
    // if (ok) {    
    //     ok = GAMCheckSignalProperties(*this, "CYCCNT", InputSignals, UnsignedInteger32Bit, 0u, 1u, signalIdx);
    //     if (ok) {
    //         INPUT_CYCCNT = (uint32*) GetInputSignalMemory(signalIdx);
    //     } else {
    //         REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal CYCCNT ");
    //     }
    // }
    // if (ok) {    
    //     ok = GAMCheckSignalProperties(*this, "state", InputSignals, UnsignedInteger8Bit, 0u, 1u, signalIdx);
    //     if (ok) {
    //         INPUT_state = (uint8*) GetInputSignalMemory(signalIdx);
    //     } else {
    //         REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal state ");
    //     }
    // }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "encoder_counter", InputSignals, UnsignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            INPUT_encoder_counter = (uint32*) GetInputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for input signal encoder_counter ");
        }
    }

    if (ok) {
        uint32 nOfOutputSignals = GetNumberOfOutputSignals();
        ok = (nOfOutputSignals == 6u); // Will need to be changed if any output signals are added or removed
        if (!ok) {
            REPORT_ERROR(ErrorManagement::ParametersError, "%s::Number of output signals must be 3", gam_name.Buffer());
        }
    }
    
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "state", OutputSignals, UnsignedInteger8Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_state = (uint8*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal State");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "rotor_control_target_steps", OutputSignals, SignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_rotor_control_target_steps = (int32*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal rotor_control_target_steps");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "gpioState", OutputSignals, UnsignedInteger8Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_gpioState = (uint8*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal gpioState");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "L6474_Board_Pwm1Period", OutputSignals, UnsignedInteger32Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_L6474_Board_Pwm1Period = (uint32*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal L6474_Board_Pwm1Period");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "break_Control_Loop", OutputSignals, UnsignedInteger8Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_break_Control_Loop = (uint8*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal break_Control_Loop");
        }
    }
    if (ok) {    
        ok = GAMCheckSignalProperties(*this, "encoder_position", OutputSignals, Float32Bit, 0u, 1u, signalIdx);
        if (ok) {
            OUTPUT_encoder_position = (float32*) GetOutputSignalMemory(signalIdx);
        } else {
            REPORT_ERROR(ErrorManagement::InitialisationError, "Signal properties check failed for output signal encoder_position");
        }
    }


    return ok;
}


/*
 * PID Controller with low pass filter operating on derivative component
 */

void InvertedPendulumGAM::pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float * current_error, float * sample_period, float * Deriv_Filt) {

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


/*
 * Configure system based on user selection
 */

void InvertedPendulumGAM::user_configuration(void){

	enable_rotor_actuator_test = 0;
	enable_rotor_actuator_control = 0;
	enable_encoder_test = 0;
	enable_rotor_actuator_high_speed_test = 0;
	enable_motor_actuator_characterization_mode = 0;
	enable_full_sysid = 0;

	enable_rotor_tracking_comb_signal = 0;
	rotor_track_comb_amplitude = 0;
	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;

//############## Jawad Modification  -->> ###############################
	enable_state_feedback = 0;
	select_suspended_mode = 0;
	proportional = 		PRIMARY_PROPORTIONAL_MODE_1;
	integral = 			PRIMARY_INTEGRAL_MODE_1;
	derivative = 		PRIMARY_DERIVATIVE_MODE_1;
	rotor_p_gain = 		SECONDARY_PROPORTIONAL_MODE_1;
	rotor_i_gain = 		SECONDARY_INTEGRAL_MODE_1;
	rotor_d_gain = 		SECONDARY_DERIVATIVE_MODE_1;
	max_speed =(u_int16_t)MAX_SPEED_MODE_1;
	min_speed =(u_int16_t)MIN_SPEED_MODE_1;
	enable_rotor_plant_design = 0;
	enable_rotor_plant_gain_design = 0;
	enable_rotor_position_step_response_cycle = 0;
	enable_pendulum_position_impulse_response_cycle = 0;
	enable_rotor_chirp = 0;
	enable_mod_sin_rotor_tracking = 1;
	enable_angle_cal = 1;
	enable_swing_up = 1;
	
}

void InvertedPendulumGAM::restart_execution(){
    if (ACCEL_CONTROL == 1) {
        desired_pwm_period = 0u;
        current_pwm_period = 0u;
    }
    state = STATE_INITIALIZATION;
}

bool InvertedPendulumGAM::Execute() {

    //MARTe::uint8 state                    = *INPUT_state;
    MARTe::uint32 message_count           =  *INPUT_message_count;
    MARTe::uint32 break_Control_Loop      =  *INPUT_break_Control_Loop;
    //reset all outputs
   *OUTPUT_rotor_control_target_steps=0;
   *OUTPUT_gpioState = UNKNOW_DIR;
   *OUTPUT_L6474_Board_Pwm1Period = 0;
   *OUTPUT_break_Control_Loop = 0;

    bool  ret = true;
    if( message_count > 0u){

        if( break_Control_Loop == 1u){//if break command is received
             restart_execution();
        }
       
        if( state == STATE_INITIALIZATION){
            control_logic_State_Initialization();
            state = STATE_PENDULUM_STABLIZATION;
            control_logic_State_PendulumStablisation_Prepare();
        }
        if( state == STATE_PENDULUM_STABLIZATION){
            ret = control_logic_State_PendulumStablisation();
            if( ret ){
                state = STATE_SWING_UP;
                control_logic_State_SwingingUp_Prepare();
            }
        }
        if( state == STATE_SWING_UP){ //initialisation state
            ret = control_logic_State_SwingingUp();
            if( ret ){//state change
                state = STATE_MAIN;
                control_logic_State_Main_Prepare();
            }
        }
        if( state == STATE_MAIN ) {   // main state 
            ret = control_logic_State_Main();
            if( !ret ){//state change
                restart_execution();
            }
        }
    }
    
    *OUTPUT_state = state;
    *OUTPUT_encoder_position = encoder_position_steps;        
    return true;
}

bool oppositeSigns(int x, int y) {
	return ((x ^ y) < 0);
}

int InvertedPendulumGAM::encoder_position_read(int *encoder_position, int encoder_position_init) {

	//cnt3 = __HAL_TIM_GET_COUNTER(htim3);

    uint32 cnt3 = *INPUT_encoder_counter;

	if (cnt3 >= 32768u) {
		*encoder_position = (int) (cnt3);
		*encoder_position = *encoder_position - 65536;
	} else {
		*encoder_position = (int) (cnt3);
	}

	range_error = 0;
	if (*encoder_position <= -32768) {
		range_error = -1;
		*encoder_position = -32768;
	}
	if (*encoder_position >= 32767) {
		range_error = 1;
		*encoder_position = 32767;
	}

	*encoder_position = *encoder_position - encoder_position_init;

	/*
	 *  Detect if we passed the bottom, then re-arm peak flag
	 *  oppositeSigns returns true when we pass the bottom position
	 */


	if (oppositeSigns(*encoder_position, previous_encoder_position))
	{
		peaked = false;
		zero_crossed = true;
	}

	if (!peaked) // We don't need to evaluate anymore if we hit a maximum when we're still in downward motion and didn't cross the minimum
	{
		// Add global maximum
		if (abs(*encoder_position) >= abs(global_max_encoder_position))
		{
			global_max_encoder_position = *encoder_position;
		}
		// Check if new maximum
		if (abs(*encoder_position) >= abs(max_encoder_position))
		{
			max_encoder_position = *encoder_position;
		}
		else
		{
			// We are at the peak and disable further checks until we traversed the minimum position again
			peaked = true;
		    handled_peak = false;
		}
	}

	previous_encoder_position = *encoder_position;


	return range_error;
}


CLASS_REGISTER(InvertedPendulumGAM, "1.0");

} // namespace MFI
