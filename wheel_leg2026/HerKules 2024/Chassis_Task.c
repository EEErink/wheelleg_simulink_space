#include "Chassis_Task.h"
#include "bsp_buzzer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "detect_task.h"
#include "remote_control.h"
#include "referee.h"
#include "can.h"
#include "bsp_can.h"
#include "arm_math.h"
#include "string.h"
#include "Kalman_Mix.h"
#include "INS_Task.h"
#include "math.h"
#include "VMC.h"
#include "mpc.h"
#include "lqr.h"
#include "user_lib.h"


#ifndef PI
#define PI 3.14159265358979f
#endif
#ifndef FOLLOW_DEADBAND
#define FOLLOW_DEADBAND 0.05f
#endif
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef square
#define square(x) ((x)*(x))
#endif

#define ABS(x) (((x) > 0) ? (x) : (-(x)))
#define LimitMax(input, max)   \
{                              \
	if ((input) > max)       \
	{                      \
			input = max;   \
	}                      \
	else if ((input) < -max) \
	{                      \
			input = -max;  \
	}                      \
}
#define LimitOutput(input, min, max )   \
{										\
	if( input < min )					\
		input = min;					\
	else if( input > max )				\
		input = max;					\
}

#define SIGN(x) ((x)>0 ? 1 : ((x)<0? -1: 0))


uint8_t use_mpc = 0, abnormal_debug = 0;
chassis_control_t chassis_control;
kalman2_state Body_Speed_KF;
// ------------------ PID info ------------------ 
fp32 roll_PD[2]        = {100.0, 20.0}; // 200, 45
fp32 coordinate_PD[2]  = { 10.0f, 0.5f }; //15.0f,1.0f
fp32 yaw_PD_test[2]    = { 20.0f, 180.0f };
fp32 stand_PD[2]       = { 200.0f, 50.0f };
fp32 jump_stand_PD[2]       = { 10000.0f, 150.0f };
fp32 suspend_stand_PD[2] = { 100.0f, 30.0f };
// fp32 stand_PD[2]={150.0f, 20.0f};//1N/cm
fp32 suspend_foot_speed_p = 10.0f;
// fp32 High_Offset = 0.20;
fp32 High_Offset = 0.06;
fp32 Moving_High_Offset = 0.0f;
uint8_t last_is_update;
fp32 IDEAL_PREPARING_STAND_JUMPING_ANGLE = 0.6f;

extern fp32 LQR[4][10];

fp32 suspend_LQR[2][6] = {	
	20.0f,5.0f,	0.0f,	0.0f,	0.0f,	0.0f,
	0,	0,  0,  0,	  0,	0
};
fp32 x_set[10];
fp32 debug_2 = 1,upper_bound, stepp = 0.2;
fp32 debug_flag, last_debug_flag;
fp32 HIGH_HIGH = 0.35f;

fp32 SIT_HIGH = 0.11f;
fp32 NORMAL_HIGH = 0.18f; //16

fp32 lower_dec_speed = 1.0f;
uint8_t mpc_cnt = 0, mpc_period = 9;
float alpha_dx = 0.95f;
fp32 rollP, rollD, roll_angle_deadband = 0.0f, roll_gyro_deadband = 0.0f, leg_dlength_deadband = 0.0f;
fp32 rotate_move_threshold = 45.0f, rotate_speed = 12.0f, fake_sign = 1.0f, rc_sign = 1.0f, rotate_move_scale = 1.0f,normal_move_scale = 2.0f, offset_k = 0.31f, fly_speed = 2.5f;
fp32 rc_angle, rc_angle_temp, X_speed, Y_speed, delta_theta, delta_theta_temp, rotate_move_offset, normalized_speed, temp_max_spd, acc_step = 1.3f, dec_step = 0.1f;
fp32 move_scale_list[11] =     { 0.0, 1.0, 1.5, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
fp32 cap_move_scale_list[11] = { 0.0, 1.5, 2.0, 2.5, 2.5, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};
fp32 rotate_speed_list[11] = {0.0, 5.0, 5.3, 5.6, 6.0, 6.0, 7.0, 8.0, 9.0, 10.0, 12.0}; 
fp32 rotate_move_scale_list[11] = {0.0, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8};
int speed_sign = 0, target_speed_sign = 0;
uint8_t robot_level = 1;
ext_robot_status_t *robot;
fp32 TK_x_p = -10.0f, TK_y_p = 10.0f, TK_y_d = 3.0f, reducing_p = 120.0f;
fp32 stablize_foot_speed_threshold = 1.2f, stablize_yaw_speed_threshold = 1.5f;
uint8_t no_follow_flag, follow_angle;

enum Chassis_Mode chassis_mode;


// ------------------ function definition ------------------ 
void Chassis_Init( chassis_control_t *init );
void Chassis_Data_Update( chassis_control_t *fdb );
void Chassis_Status_Detect( chassis_control_t *detect );
void Chassis_Mode_Set( chassis_control_t *mode_set );
void Chassis_Mode_Change_Control_Transit( chassis_control_t *chassis_mode_change );
void Target_Value_Set( chassis_control_t *target_value_set );
void Chassis_Torque_Calculation( chassis_control_t *bl_ctrl );
void Chassis_Torque_Combine( chassis_control_t *bl_ctrl );
void Motor_CMD_Send(chassis_control_t *CMD_Send);
void Joint_Motor_to_Init_Pos(void);
void Motor_Zero_CMD_Send(void);
void HT_Motor_zero_set(void);

int16_t set1, set2;
uint16_t launching_time = 0;

void Chassis_Task(void const * argument)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	Chassis_Init(&chassis_control);
	vTaskDelay(100);
	while(1)
	{
		Chassis_Data_Update(&chassis_control);
		Chassis_Status_Detect(&chassis_control);
		Chassis_Mode_Set(&chassis_control);
		Chassis_Mode_Change_Control_Transit(&chassis_control);
		Target_Value_Set(&chassis_control);
		Chassis_Torque_Calculation(&chassis_control);
		Chassis_Torque_Combine(&chassis_control);
		Motor_CMD_Send(&chassis_control);
		vTaskDelay(1);
	}
}


void Chassis_Init( chassis_control_t *init )
{
	// ------------------ Set HT Zero Point ------------------ 
	buzzer_on(75, 60);
	vTaskDelay(100);
	HT_Motor_zero_set();
	buzzer_off();

	Motor_Zero_CMD_Send();

	kalman_second_order_init(&Body_Speed_KF);

	// ------------------ Ptr Set ------------------ 
	init->joint_motor_1.motor_measure = get_HT_motor_measure_point(0);
	init->joint_motor_2.motor_measure = get_HT_motor_measure_point(1);
	init->joint_motor_3.motor_measure = get_HT_motor_measure_point(2);
	init->joint_motor_4.motor_measure = get_HT_motor_measure_point(3);
	// init->foot_motor_L.motor_measure = get_BMmotor_measure_point(0);
	// init->foot_motor_R.motor_measure = get_BMmotor_measure_point(1);
	init->chassis_rc_ctrl = get_Gimabl_control_point();
	init->chassis_posture_info.chassis_INS_angle_point = get_INS_angle_point();
	init->chassis_posture_info.chassis_INS_gyro_point = get_gyro_data_point();
	init->chassis_posture_info.chassis_INS_accel_point = get_acc_data_point();

	/*
	first_order_filter_init(&rc_filter, 0.002f, rc_filt_numb);
	first_order_filter_init(&tl_filter, 0.002f, tl_numb);
	first_order_filter_init(&tl_run_filter, 0.002f, tl_numb);
    OLS_Init(&OLS_S0_L,2);
	OLS_Init(&OLS_S0_R,2);
	*/

	// ------------------ Mode Set ------------------ 
	init->mode.chassis_mode = init->mode.last_chassis_mode = DISABLE_CHASSIS;
	init->mode.chassis_balancing_mode = init->mode.last_chassis_balancing_mode = NO_FORCE;
	init->mode.sport_mode = init->mode.last_sport_mode = NORMAL_MOVING_MODE;
	init->joint_motor_1.motor_mode = init->joint_motor_1.last_motor_mode = MOTOR_NO_FORCE;
	init->joint_motor_2.motor_mode = init->joint_motor_2.last_motor_mode = MOTOR_NO_FORCE;
	init->joint_motor_3.motor_mode = init->joint_motor_3.last_motor_mode = MOTOR_NO_FORCE;
	init->joint_motor_4.motor_mode = init->joint_motor_4.last_motor_mode = MOTOR_NO_FORCE;
	init->foot_motor_L.motor_mode = init->foot_motor_L.last_motor_mode = MOTOR_NO_FORCE;
	init->foot_motor_R.motor_mode = init->foot_motor_R.last_motor_mode = MOTOR_NO_FORCE;

	// ------------------ Initialize HT Motor ------------------ 
	init->joint_motor_1.position_offset = init->joint_motor_1.motor_measure->ecd;
	init->joint_motor_2.position_offset = init->joint_motor_2.motor_measure->ecd;
	init->joint_motor_3.position_offset = init->joint_motor_3.motor_measure->ecd;
	init->joint_motor_4.position_offset = init->joint_motor_4.motor_measure->ecd;
	init->joint_motor_1.position = (init->joint_motor_1.motor_measure->ecd -init->joint_motor_1.position_offset) + LEG_OFFSET;
	init->joint_motor_2.position = (init->joint_motor_2.motor_measure->ecd -init->joint_motor_2.position_offset) - LEG_OFFSET;
	init->joint_motor_3.position = (init->joint_motor_3.motor_measure->ecd -init->joint_motor_3.position_offset) + LEG_OFFSET;
	init->joint_motor_4.position = (init->joint_motor_4.motor_measure->ecd -init->joint_motor_4.position_offset) - LEG_OFFSET;

	init->flag_info.init_flag = 1;
	Chassis_Data_Update(init);
	init->flag_info.init_flag = 0;

	init->foot_motor_L.distance_offset = init->foot_motor_L.distance;
	init->foot_motor_R.distance_offset = init->foot_motor_R.distance;
	
	Power_Init(init);

	// ------------------ MPC init ------------------
	MPC_Init();
}

void Chassis_Data_Update( chassis_control_t *fdb )
{
	robot = get_robot_status_point();
	robot_level = robot->robot_level;
	// robot_level = 1;
	// -------------------- Update foot measure --------------------
	fdb->foot_motor_L.motor_measure.ecd = (uint16_t)motor_BM[0].ecd;
	fdb->foot_motor_L.motor_measure.last_ecd = (uint16_t)motor_BM[0].last_ecd;
	fdb->foot_motor_L.motor_measure.speed_rpm = (int16_t)motor_BM[0].speed_rpm;
	fdb->foot_motor_L.torque_get = motor_BM[0].given_current*55.0f/32767.0f;

	fdb->foot_motor_R.motor_measure.ecd = (uint16_t)motor_BM[1].ecd;
	fdb->foot_motor_R.motor_measure.last_ecd = (uint16_t)motor_BM[1].last_ecd;
	fdb->foot_motor_R.motor_measure.speed_rpm = (int16_t)motor_BM[1].speed_rpm;
	fdb->foot_motor_R.torque_get = motor_BM[1].given_current*55.0f/32767.0f;

	//  ------------------ Update mode info ------------------ 
	fdb->mode.last_chassis_mode                = fdb->mode.chassis_mode;
	fdb->mode.last_chassis_balancing_mode      = fdb->mode.chassis_balancing_mode;
	fdb->mode.last_sport_mode                  = fdb->mode.sport_mode;
	fdb->mode.last_jumping_mode                = fdb->mode.jumping_mode;
	fdb->mode.last_jumping_stage               = fdb->mode.jumping_stage;
	fdb->mode.last_chassis_high_mode           = fdb->mode.chassis_high_mode;
	fdb->flag_info.last_static_flag            = fdb->flag_info.static_flag;
	fdb->flag_info.last_moving_flag            = fdb->flag_info.moving_flag;
	last_debug_flag                            = debug_flag;
	fdb->flag_info.last_overpower_warning_flag = fdb->flag_info.overpower_warning_flag;
	fdb->flag_info.last_stablize_high_flag     = fdb->flag_info.stablize_high_flag;

	fdb->flag_info.last_suspend_flag_L         = fdb->flag_info.suspend_flag_L;
	fdb->flag_info.last_suspend_flag_R         = fdb->flag_info.suspend_flag_R;

	fdb->joint_motor_1.last_motor_mode = fdb->joint_motor_1.motor_mode;
	fdb->joint_motor_2.last_motor_mode = fdb->joint_motor_2.motor_mode;
	fdb->joint_motor_3.last_motor_mode = fdb->joint_motor_3.motor_mode;
	fdb->joint_motor_4.last_motor_mode = fdb->joint_motor_4.motor_mode;
	fdb->foot_motor_L.last_motor_mode  = fdb->foot_motor_L.motor_mode;
	fdb->foot_motor_R.last_motor_mode  = fdb->foot_motor_R.motor_mode;

	// ------------------ Update IMU info ------------------ 
	fdb->chassis_posture_info.pitch_angle = *(fdb->chassis_posture_info.chassis_INS_angle_point  + INS_PITCH_ADDRESS_OFFSET);
	fdb->chassis_posture_info.roll_angle  = *(fdb->chassis_posture_info.chassis_INS_angle_point  + INS_ROLL_ADDRESS_OFFSET);
	fdb->chassis_posture_info.yaw_angle   = *(fdb->chassis_posture_info.chassis_INS_angle_point  + INS_YAW_ADDRESS_OFFSET) + yaw_count * PI2;
	fdb->chassis_posture_info.pitch_gyro  = *(fdb->chassis_posture_info.chassis_INS_gyro_point  + INS_PITCH_ADDRESS_OFFSET);
	fdb->chassis_posture_info.roll_gyro =   *(fdb->chassis_posture_info.chassis_INS_gyro_point  + INS_ROLL_ADDRESS_OFFSET);
  	fdb->chassis_posture_info.yaw_gyro    = *(fdb->chassis_posture_info.chassis_INS_gyro_point  + INS_YAW_ADDRESS_OFFSET);

	// ------------------ Update HT Motor info ------------------ 
	fdb->joint_motor_1.position = (fdb->joint_motor_1.motor_measure->ecd - fdb->joint_motor_1.position_offset) + LEG_OFFSET;
	fdb->joint_motor_2.position = (fdb->joint_motor_2.motor_measure->ecd - fdb->joint_motor_2.position_offset) - LEG_OFFSET;
	fdb->joint_motor_3.position = (fdb->joint_motor_3.motor_measure->ecd - fdb->joint_motor_3.position_offset) + LEG_OFFSET;
	fdb->joint_motor_4.position = (fdb->joint_motor_4.motor_measure->ecd - fdb->joint_motor_4.position_offset) - LEG_OFFSET;

	fdb->joint_motor_1.velocity = fdb->joint_motor_1.motor_measure->speed_rpm;
	fdb->joint_motor_2.velocity = fdb->joint_motor_2.motor_measure->speed_rpm;
	fdb->joint_motor_3.velocity = fdb->joint_motor_3.motor_measure->speed_rpm;
	fdb->joint_motor_4.velocity = fdb->joint_motor_4.motor_measure->speed_rpm;

	fdb->joint_motor_1.torque_get = 0.95f * fdb->joint_motor_1.torque_get + 0.05f * fdb->joint_motor_1.motor_measure->real_torque;
	fdb->joint_motor_2.torque_get = 0.95f * fdb->joint_motor_2.torque_get + 0.05f * fdb->joint_motor_2.motor_measure->real_torque;
	fdb->joint_motor_3.torque_get = 0.95f * fdb->joint_motor_3.torque_get + 0.05f * fdb->joint_motor_3.motor_measure->real_torque;
	fdb->joint_motor_4.torque_get = 0.95f * fdb->joint_motor_4.torque_get + 0.05f * fdb->joint_motor_4.motor_measure->real_torque;

	// ------------------ Update LK Motor info ------------------ 
	fdb->foot_motor_L.last_position = fdb->foot_motor_L.position;
	fdb->foot_motor_R.last_position = fdb->foot_motor_R.position;
	fdb->foot_motor_L.position = fdb->foot_motor_L.motor_measure.ecd * MOTOR_ECD_TO_RAD;// Angle (rad)
	fdb->foot_motor_R.position = fdb->foot_motor_R.motor_measure.ecd * MOTOR_ECD_TO_RAD;

	if( fdb->flag_info.init_flag != 1 ) // Init_flag = 0: after Init fdb 如果不是在init里面调用就�?�圈
	{
		if( (fdb->foot_motor_L.last_position - fdb->foot_motor_L.position ) > HALF_POSITION_RANGE )
			fdb->foot_motor_L.turns++;		
		else if( (fdb->foot_motor_L.last_position - fdb->foot_motor_L.position ) < -HALF_POSITION_RANGE )
			fdb->foot_motor_L.turns--;	
		if( ( fdb->foot_motor_R.last_position - fdb->foot_motor_R.position ) > HALF_POSITION_RANGE )
			fdb->foot_motor_R.turns--;
		else if( ( fdb->foot_motor_R.last_position - fdb->foot_motor_R.position ) < -HALF_POSITION_RANGE )
			fdb->foot_motor_R.turns++;
	}
	fdb->foot_motor_L.distance	= ( fdb->foot_motor_L.position/PI2 + fdb->foot_motor_L.turns ) * WHEEL_PERIMETER - fdb->foot_motor_L.distance_offset;
	fdb->foot_motor_R.distance  = ( -fdb->foot_motor_R.position/PI2 + fdb->foot_motor_R.turns ) * WHEEL_PERIMETER - fdb->foot_motor_R.distance_offset;
	fdb->chassis_posture_info.foot_distance = ( fdb->foot_motor_L.distance + fdb->foot_motor_R.distance ) /2.0f;
	
	fdb->foot_motor_L.speed = fdb->foot_motor_L.motor_measure.speed_rpm * PI2 * WHEEL_RADIUS / 10.0f / 60.0f; // rpm -> m/s
	fdb->foot_motor_R.speed = -fdb->foot_motor_R.motor_measure.speed_rpm * PI2 * WHEEL_RADIUS / 10.0f / 60.0f;
	fdb->chassis_posture_info.foot_speed  = ( fdb->foot_motor_L.speed + fdb->foot_motor_R.speed ) / 2.0f;

	// fdb->foot_motor_L.torque_get = fdb->foot_motor_L.motor_measure.given_current;
	// fdb->foot_motor_R.torque_get = fdb->foot_motor_R.motor_measure.given_current;
	
	// ------------------ Kalman Filter ------------------ 

	kalman_second_order_update(&Body_Speed_KF, fdb->chassis_posture_info.foot_distance, fdb->chassis_posture_info.foot_speed, temp_a);
	fdb->chassis_posture_info.foot_distance_K = Body_Speed_KF.x[0];
	fdb->chassis_posture_info.foot_speed_K    =  Body_Speed_KF.x[1];

	// ------------------ Five_Bars_to_Pendulum ------------------ 
	Forward_kinematic_solution(fdb,-fdb->joint_motor_2.position,-fdb->joint_motor_2.velocity,
		-fdb->joint_motor_1.position,-fdb->joint_motor_1.velocity,1);
	Forward_kinematic_solution(fdb,fdb->joint_motor_3.position,fdb->joint_motor_3.velocity,
		fdb->joint_motor_4.position, fdb->joint_motor_4.velocity,0);

	// ------------------ Huanzhi information update ------------------ 
	fdb->chassis_posture_info.leg_angle_L -= PI_2;
	fdb->chassis_posture_info.leg_angle_L -= fdb->chassis_posture_info.pitch_angle;
	// fdb->chassis_posture_info.leg_gyro_L  -= fdb->chassis_posture_info.pitch_gyro;
	fp32 temp_v_L = ( fdb->chassis_posture_info.leg_angle_L - fdb->chassis_posture_info.last_leg_angle_L ) / TASK_RUN_TIME;
	fdb->chassis_posture_info.leg_gyro_L = alpha_dx * temp_v_L + (1-alpha_dx) * fdb->chassis_posture_info.leg_gyro_L;
	fdb->chassis_posture_info .last_leg_angle_L = fdb->chassis_posture_info .leg_angle_L;

	fdb->chassis_posture_info.leg_angle_R -= PI_2;
	fdb->chassis_posture_info.leg_angle_R -= fdb->chassis_posture_info.pitch_angle;
	// fdb->chassis_posture_info.leg_gyro_R  -= fdb->chassis_posture_info.pitch_gyro;
	fp32 temp_v_R = ( fdb->chassis_posture_info.leg_angle_R - fdb->chassis_posture_info.last_leg_angle_R ) / TASK_RUN_TIME;
	fdb->chassis_posture_info.leg_gyro_R = alpha_dx * temp_v_R + (1-alpha_dx) * fdb->chassis_posture_info.leg_gyro_R;
	fdb->chassis_posture_info.last_leg_angle_R = fdb->chassis_posture_info .leg_angle_R;

	temp_v_L = ( fdb->chassis_posture_info.leg_length_L - fdb->chassis_posture_info.last_leg_length_L ) / TASK_RUN_TIME;
	fdb->chassis_posture_info.leg_dlength_L = alpha_dx * temp_v_L + (1-alpha_dx) * fdb->chassis_posture_info.leg_dlength_L;
	fdb->chassis_posture_info.last_leg_length_L = fdb->chassis_posture_info.leg_length_L;

	temp_v_R = ( fdb->chassis_posture_info.leg_length_R - fdb->chassis_posture_info.last_leg_length_R ) / TASK_RUN_TIME;
	fdb->chassis_posture_info.leg_dlength_R = alpha_dx * temp_v_R + (1-alpha_dx) * fdb->chassis_posture_info.leg_dlength_R;
	fdb->chassis_posture_info.last_leg_length_R = fdb->chassis_posture_info.leg_length_R;

	// ---------- Rotate and Move Info update ------------
	rotate_move_offset = offset_k * rotate_speed_list[robot_level];
	rc_sign = 1.0f;
	X_speed = fdb->chassis_rc_ctrl->X_speed;
	Y_speed = fdb->chassis_rc_ctrl->Y_speed;
	// Original rc angle
	if (X_speed == 0){
		if (Y_speed > 0)
			rc_angle_temp = PI/2;
		else if (Y_speed < 0)
			rc_angle_temp = -PI/2;
		else
			rc_angle_temp = 0;
	}
	else{
		Y_speed = fdb->chassis_rc_ctrl->Y_speed;
		rc_angle_temp = atan_tl(Y_speed/X_speed);
	}
	// Normalized speed
	if (abs(X_speed) < 0.1 ||abs(Y_speed) <0.1)
		temp_max_spd = 1.0f;
	else if (X_speed<Y_speed){
		temp_max_spd = sqrt(square((1/Y_speed)*X_speed)+1);
	}
	else{
		temp_max_spd = sqrt(square((1/X_speed)*Y_speed)+1);
	}
	normalized_speed = sqrt(X_speed*X_speed+Y_speed*Y_speed) / temp_max_spd;
	// Ref angle and sign
	if (X_speed<0) rc_sign *= -1.0f;
	rc_angle_temp += rotate_move_offset;
	if (rc_angle_temp>PI/2||rc_angle_temp<-PI/2){
		rc_sign *= -1.0f;
	}
	while (rc_angle_temp>PI/2) {
		rc_angle_temp -= PI;
	}
	while (rc_angle_temp<-PI/2){
		rc_angle_temp += PI;
	} 
	// if (X_speed < 0) rc_sign = -1.0f; 	
	rc_angle = rc_angle_temp * 180.0f / PI;
	if (fdb->chassis_rc_ctrl->fake_flag == 0) fake_sign = -1.0f;
	else fake_sign = 1.0f;
}

void Chassis_Status_Detect( chassis_control_t *detect )
{
	// ------------------ Off Ground Detect ------------------ 


	if( detect->mode.chassis_balancing_mode == BALANCING_READY && detect->mode.sport_mode!=TK_MODE)
	{
		if( ABS(detect->chassis_posture_info.pitch_angle) >= DANGER_PITCH_ANGLE )
			detect->flag_info.abnormal_flag = 1;
		else if( ABS(detect->chassis_posture_info.foot_speed_K) < MOVE_LOWER_BOUND && 
			ABS(detect->chassis_posture_info.pitch_angle)        < 0.1f &&
				detect->flag_info.abnormal_flag )
				detect->flag_info.abnormal_flag = 0;
	}
	if (abnormal_debug){
		detect->flag_info.abnormal_flag = 0;
	}

	Supportive_Force_Cal(detect,  detect->joint_motor_1.position, detect->joint_motor_2.position, 1.0f );	
	Supportive_Force_Cal(detect,  detect->joint_motor_3.position,  detect->joint_motor_4.position, 0.0f );
	
	if( detect->mode.jumping_stage == CONSTACTING_LEGS )
		detect->flag_info.suspend_flag_L = detect->flag_info.suspend_flag_R = ON_GROUND;
	else
	{
		if( detect->mode.sport_mode == JUMPING_MODE )
		{
			if( detect->chassis_posture_info.supportive_force_R <= LOWER_SUPPORT_FORCE_FOR_JUMP &&
				detect->chassis_posture_info.leg_length_R > 0.13f )
				detect->flag_info.suspend_flag_R = OFF_GROUND;
			else
				detect->flag_info.suspend_flag_L = ON_GROUND;
			if( detect->chassis_posture_info.supportive_force_L <= LOWER_SUPPORT_FORCE_FOR_JUMP &&
				detect->chassis_posture_info.leg_length_L > 0.13f )
				detect->flag_info.suspend_flag_L = OFF_GROUND;
			else 
				detect->flag_info.suspend_flag_R = ON_GROUND;
		}
		else
		{
			if( detect->chassis_posture_info.supportive_force_R <= LOWER_SUPPORT_FORCE &&
				detect->chassis_posture_info.leg_length_R > 0.13f )
				detect->flag_info.suspend_flag_R = OFF_GROUND;
			else
				detect->flag_info.suspend_flag_L = ON_GROUND;
			if( detect->chassis_posture_info.supportive_force_L <= LOWER_SUPPORT_FORCE &&
				detect->chassis_posture_info.leg_length_L > 0.13f )
				detect->flag_info.suspend_flag_L = OFF_GROUND;
			else 
				detect->flag_info.suspend_flag_R = ON_GROUND;
		}
	} 


	if( detect->flag_info.abnormal_flag == 1 &&
		( detect->flag_info.last_suspend_flag_L == ON_GROUND || detect->flag_info.last_suspend_flag_R == ON_GROUND ) &&
		( detect->flag_info.suspend_flag_L == OFF_GROUND || detect->flag_info.suspend_flag_R  == OFF_GROUND ) )
			detect->flag_info.Ignore_Off_Ground = 1;
	else if( detect->flag_info.abnormal_flag != 1 )
		detect->flag_info.Ignore_Off_Ground = 0;
	if( detect->flag_info.Ignore_Off_Ground )
	{
		detect->flag_info.suspend_flag_R = ON_GROUND;
		detect->flag_info.suspend_flag_L = ON_GROUND;
	}
	// detect->flag_info.suspend_flag_L = detect->flag_info.suspend_flag_R = ON_GROUND;
	//Moving_High_Offset

	if( ABS(detect->chassis_posture_info.foot_speed_K) > stablize_foot_speed_threshold &&
		ABS(detect->chassis_posture_info.yaw_gyro ) > stablize_yaw_speed_threshold )
		detect->flag_info.stablize_high_flag = 1;

}

void Chassis_Mode_Set( chassis_control_t *mode_set )
{
	// ----------------- Chassis Mode Update ----------------- 
	if( mode_set->chassis_rc_ctrl->mode_R == 0 || toe_is_error(GIMBAL_TOE)|| toe_is_error(BM1_TOE)||toe_is_error(BM2_TOE))
		mode_set->mode.chassis_mode = DISABLE_CHASSIS;
	else 
		mode_set->mode.chassis_mode = ENABLE_CHASSIS;
	
	if( mode_set->mode.chassis_mode == ENABLE_CHASSIS )
	{
		mode_set->joint_motor_1.motor_mode = MOTOR_FORCE;
		mode_set->joint_motor_2.motor_mode = MOTOR_FORCE;
		mode_set->joint_motor_3.motor_mode = MOTOR_FORCE;
		mode_set->joint_motor_4.motor_mode = MOTOR_FORCE;
		mode_set->foot_motor_L.motor_mode  = MOTOR_FORCE;
		mode_set->foot_motor_R.motor_mode  = MOTOR_FORCE;
	}
	else
	{
		if( mode_set->mode.chassis_balancing_mode == JOINT_REDUCING )
		{
			mode_set->joint_motor_1.motor_mode = MOTOR_FORCE;
			mode_set->joint_motor_2.motor_mode = MOTOR_FORCE;
			mode_set->joint_motor_3.motor_mode = MOTOR_FORCE;
			mode_set->joint_motor_4.motor_mode = MOTOR_FORCE;
			mode_set->foot_motor_L.motor_mode  = MOTOR_NO_FORCE;
			mode_set->foot_motor_R.motor_mode  = MOTOR_NO_FORCE;
		}
		else
		{
			mode_set->joint_motor_1.motor_mode = MOTOR_NO_FORCE;
			mode_set->joint_motor_2.motor_mode = MOTOR_NO_FORCE;
			mode_set->joint_motor_3.motor_mode = MOTOR_NO_FORCE;
			mode_set->joint_motor_4.motor_mode = MOTOR_NO_FORCE;
			mode_set->foot_motor_L.motor_mode  = MOTOR_NO_FORCE;
			mode_set->foot_motor_R.motor_mode  = MOTOR_NO_FORCE;
		}

	}

	// ----------------- Sport Mode Update ----------------- 
	if( mode_set->mode.chassis_balancing_mode == BALANCING_READY )
	{
		if( mode_set->chassis_rc_ctrl->tk_flag )
			mode_set->mode.sport_mode = TK_MODE;
		else if( mode_set->flag_info.abnormal_flag )
			mode_set->mode.sport_mode = ABNORMAL_MOVING_MODE;
		else if( mode_set->mode.sport_mode == JUMPING_MODE && mode_set->mode.jumping_stage != FINISHED )
			mode_set->mode.sport_mode = JUMPING_MODE;
		else if( mode_set->chassis_rc_ctrl->jump_flag )
			mode_set->mode.sport_mode = JUMPING_MODE;
		else if( mode_set->chassis_rc_ctrl->cap_flag )
			mode_set->mode.sport_mode = CAP_MODE;
		else if( mode_set->chassis_rc_ctrl->fly_flag )
			mode_set->mode.sport_mode = FLY_MODE;
		else 
			mode_set->mode.sport_mode = NORMAL_MOVING_MODE;
	}
	else
		mode_set->mode.sport_mode = NONE;

	// ----------------- Rotation Flag --------------------
	static uint8_t last_rotation_flag = 0;
	last_rotation_flag = mode_set->flag_info.rotation_flag;
	mode_set->flag_info.rotation_flag = mode_set->chassis_rc_ctrl->mode_R == 4; //Rotate
	if (!last_rotation_flag && mode_set->flag_info.rotation_flag){
		for (int i = 0; i < 11; ++i)
			rotate_speed_list[i] = - rotate_speed_list[i];
	}

	// ----------------- No Follow Flag ----------------------
	if (toe_is_error(GIMBAL_TOE)) no_follow_flag = 1;
	else no_follow_flag = mode_set->chassis_rc_ctrl->mode_R == 3; //No Follow
	
	// ---------------- Moving Flag ----------------------------
	if( mode_set->chassis_rc_ctrl->X_speed != 0.0f )
	{
		mode_set->flag_info.controlling_flag = 1;
		mode_set->flag_info.set_pos_after_moving = 1;
	}	
	else 
		mode_set->flag_info.controlling_flag = 0;
		
	if( ABS(mode_set->chassis_posture_info.foot_speed_K) <= 0.05f &&
		!mode_set->flag_info.controlling_flag &&
		mode_set->flag_info.set_pos_after_moving )  					// maybe bug
		{
			mode_set->flag_info.moving_flag = 0;
			mode_set->flag_info.set_pos_after_moving = 0;
		}
	else 
		mode_set->flag_info.moving_flag = 1;


}

uint8_t reduce_flag = 0;
fp32 reduce_high, high_offset = 0.2f;
fp32 debug_1 = 0.995;
void Chassis_Mode_Change_Control_Transit( chassis_control_t *chassis_mode_change )
{
	if( chassis_mode_change->mode.chassis_mode == ENABLE_CHASSIS && chassis_mode_change->mode.last_chassis_mode == DISABLE_CHASSIS )
		chassis_mode_change->mode.chassis_balancing_mode = FOOT_LAUNCHING;

	if( chassis_mode_change->mode.chassis_balancing_mode == FOOT_LAUNCHING && 
		ABS(chassis_mode_change->chassis_posture_info.pitch_angle) < EXIT_PITCH_ANGLE )
		chassis_mode_change->mode.chassis_balancing_mode = JOINT_LAUNCHING;
	else if( chassis_mode_change->mode.chassis_balancing_mode == JOINT_LAUNCHING && (
		(NORMAL_HIGH - chassis_mode_change->chassis_posture_info.leg_length_L) < 0.03f ||
		(NORMAL_HIGH - chassis_mode_change->chassis_posture_info.leg_length_R) < 0.03f) )
		chassis_mode_change->mode.chassis_balancing_mode = BALANCING_READY;
	else if( chassis_mode_change->mode.chassis_balancing_mode == BALANCING_READY &&
		chassis_mode_change->mode.chassis_mode == DISABLE_CHASSIS ){
			if (!reduce_flag) {
				reduce_high = chassis_mode_change->chassis_posture_info.ideal_high + high_offset;
				reduce_flag = 1;
			}
			chassis_mode_change->mode.chassis_balancing_mode = JOINT_REDUCING;
		}
	else if( chassis_mode_change->mode.chassis_balancing_mode == JOINT_REDUCING && (
		ABS( chassis_mode_change->chassis_posture_info.leg_length_L - SIT_HIGH ) < 0.001f ||
		ABS( chassis_mode_change->chassis_posture_info.leg_length_R - SIT_HIGH ) < 0.001f ) ){
			chassis_mode_change->mode.chassis_balancing_mode = NO_FORCE;
			reduce_flag = 0;
		}

	if( chassis_mode_change->mode.sport_mode == JUMPING_MODE && chassis_mode_change->mode.last_sport_mode != JUMPING_MODE )
	{
		chassis_mode_change->mode.jumping_mode = STANDING_JUMP;
	}
	else if( chassis_mode_change->mode.sport_mode != JUMPING_MODE )
		chassis_mode_change->mode.jumping_mode = NOT_DEFINE;
	

	if( chassis_mode_change->mode.jumping_mode == MOVING_JUMP )
	{
		if( chassis_mode_change->mode.jumping_stage == READY_TO_JUMP )
			chassis_mode_change->mode.jumping_stage = CONSTACTING_LEGS;
		else if( chassis_mode_change->mode.jumping_stage == CONSTACTING_LEGS &&
			chassis_mode_change->chassis_posture_info.leg_length_L <= 0.13f )
				chassis_mode_change->mode.jumping_stage = EXTENDING_LEGS;
		else if( chassis_mode_change->mode.jumping_stage == EXTENDING_LEGS  && 
			chassis_mode_change->chassis_posture_info.leg_length_L >= 0.30f )
				chassis_mode_change->mode.jumping_stage = CONSTACTING_LEGS_2;
		else if( chassis_mode_change->mode.jumping_stage == CONSTACTING_LEGS_2  && 
			chassis_mode_change->chassis_posture_info.leg_length_L <= 0.13f )
				chassis_mode_change->mode.jumping_stage = PREPARING_LANDING;
		else if( chassis_mode_change->mode.jumping_stage == PREPARING_LANDING && 
			chassis_mode_change->flag_info.suspend_flag_R == ON_GROUND &&
			chassis_mode_change->flag_info.suspend_flag_L == ON_GROUND )
				chassis_mode_change->mode.jumping_stage = FINISHED;
		else if( chassis_mode_change->mode.jumping_stage == FINISHED )
				chassis_mode_change->mode.jumping_stage = READY_TO_JUMP;
	}
	else if( chassis_mode_change->mode.jumping_mode == STANDING_JUMP )
	{
		if( chassis_mode_change->mode.jumping_stage == READY_TO_JUMP )
			chassis_mode_change->mode.jumping_stage = PREPARING_STAND_JUMPING;
		else if(chassis_mode_change->mode.jumping_stage == PREPARING_STAND_JUMPING &&
			ABS(chassis_mode_change->chassis_posture_info.leg_angle_L + IDEAL_PREPARING_STAND_JUMPING_ANGLE ) < stepp )
			chassis_mode_change->mode.jumping_stage = EXTENDING_LEGS;
		else if( chassis_mode_change->mode.jumping_stage == EXTENDING_LEGS  && 
			chassis_mode_change->chassis_posture_info.leg_length_L >= 0.30f )
				chassis_mode_change->mode.jumping_stage = CONSTACTING_LEGS_2;
		else if( chassis_mode_change->mode.jumping_stage == CONSTACTING_LEGS_2  && 
			chassis_mode_change->chassis_posture_info.leg_length_L <= 0.13f )
				chassis_mode_change->mode.jumping_stage = PREPARING_LANDING;
		else if( chassis_mode_change->mode.jumping_stage == PREPARING_LANDING && 
			chassis_mode_change->flag_info.suspend_flag_R == ON_GROUND &&
			chassis_mode_change->flag_info.suspend_flag_L == ON_GROUND )
				chassis_mode_change->mode.jumping_stage = FINISHED;
		else if( chassis_mode_change->mode.jumping_stage == FINISHED )
				chassis_mode_change->mode.jumping_stage = READY_TO_JUMP;
	}
	else
		chassis_mode_change->mode.jumping_stage = FINISHED; 


	if( chassis_mode_change->flag_info.stablize_high_flag == 1 &&
		Moving_High_Offset >= 0.2 &&
		chassis_mode_change->chassis_posture_info.yaw_gyro <= ( stablize_yaw_speed_threshold - 0.5f ) )
			chassis_mode_change->flag_info.stablize_high_flag = 0;
}

fp32 HIGH_SWITCH = 36.0f;

void Target_Value_Set( chassis_control_t *target_value_set )
{
	// ------------------------------------
	if( target_value_set->flag_info.stablize_high_flag == 1 )
		if( Moving_High_Offset < 0.2 )
			Moving_High_Offset += 0.001f;

	if( target_value_set->flag_info.stablize_high_flag == 0 )
		Moving_High_Offset = 0.0f;

	// --------- X Speed Set ----------
	if( target_value_set->mode.sport_mode             != NONE                 && 
		target_value_set->flag_info.suspend_flag_L     == ON_GROUND &&
		target_value_set->flag_info.suspend_flag_R     == ON_GROUND )
	{
		if (!target_value_set->flag_info.rotation_flag) {// Normal move
			if(toe_is_error(REFEREE_TOE)){
				target_value_set->chassis_posture_info.foot_speed_set = fake_sign * target_value_set->chassis_rc_ctrl->X_speed * normal_move_scale ;

			} else {
				// target_speed_sign = (fake_sign * target_value_set->chassis_rc_ctrl->X_speed * move_scale_list[robot_level]) > 0 ? 1: -1;
				target_speed_sign = SIGN(fake_sign * target_value_set->chassis_rc_ctrl->X_speed * move_scale_list[robot_level]);					
				target_value_set->chassis_posture_info.foot_speed_set = target_value_set->chassis_posture_info.foot_speed_K + target_speed_sign * acc_step;

				if (target_value_set->mode.sport_mode == FLY_MODE) 
					target_value_set->chassis_posture_info.foot_speed_set = target_speed_sign * min(ABS(target_value_set->chassis_rc_ctrl->X_speed * fly_speed),ABS(target_value_set->chassis_posture_info.foot_speed_set));
				else if (target_value_set->mode.sport_mode == CAP_MODE)
					target_value_set->chassis_posture_info.foot_speed_set = target_speed_sign * min(ABS(target_value_set->chassis_rc_ctrl->X_speed * cap_move_scale_list[robot_level]),ABS(target_value_set->chassis_posture_info.foot_speed_set));
				else
					target_value_set->chassis_posture_info.foot_speed_set = target_speed_sign * min(ABS(target_value_set->chassis_rc_ctrl->X_speed * move_scale_list[robot_level]),ABS(target_value_set->chassis_posture_info.foot_speed_set));

				// if( target_value_set->chassis_posture_info.leg_length_L > 0.22f || target_value_set->chassis_posture_info.leg_length_R > 0.22f )
				// {
				// 	target_value_set->chassis_posture_info.foot_speed_set /= 2.0f;
				// 	debug_2++;
				// }	
			}
		
		}
		else // Rotate and move
		{
			delta_theta_temp = target_value_set->chassis_rc_ctrl->W_angle - rc_angle;
			if (delta_theta_temp>PI/2) delta_theta_temp -= PI;
			else if (delta_theta_temp<-PI/2) delta_theta_temp += PI;
			delta_theta = abs(delta_theta_temp);
			if (delta_theta<rotate_move_threshold)
				target_value_set->chassis_posture_info.foot_speed_set
				= ((rotate_move_threshold-delta_theta)/rotate_move_threshold)
				 * rc_sign * fake_sign * normalized_speed * rotate_move_scale_list[robot_level];
			else target_value_set->chassis_posture_info.foot_speed_set = 0;
		}

	}
	else
		target_value_set->chassis_posture_info.foot_speed_set = 0;

	// --------- Distance Set ---------
	if( target_value_set->mode.chassis_balancing_mode == NO_FORCE || target_value_set->mode.sport_mode == TK_MODE)
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
	else if( target_value_set->mode.sport_mode == ABNORMAL_MOVING_MODE )
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_set;
	else if( target_value_set->flag_info.suspend_flag_R == OFF_GROUND ||
		target_value_set->flag_info.suspend_flag_L == OFF_GROUND )
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
	else if( target_value_set->flag_info.controlling_flag )
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;
	else if( !target_value_set->flag_info.moving_flag && 
			target_value_set->mode.chassis_balancing_mode == BALANCING_READY &&
			target_value_set->flag_info.last_moving_flag )
		target_value_set->chassis_posture_info.foot_distance_set = target_value_set->chassis_posture_info.foot_distance_K;



	// --------- Y Speed & Angle Set --------- 
	if( target_value_set->mode.sport_mode             != NONE    && 
		target_value_set->flag_info.suspend_flag_L      == ON_GROUND &&
		target_value_set->flag_info.suspend_flag_R      == ON_GROUND )
	{
	 	if( target_value_set->flag_info.rotation_flag == 1 )
	 	{
	 	 	target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle;
	 	 	target_value_set->chassis_posture_info.yaw_gyro_set = rotate_speed_list[robot_level];
	 	} 
	 	else
	 	{
			if (target_value_set->chassis_rc_ctrl->W_angle<-FOLLOW_DEADBAND) 
				target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle + (target_value_set->chassis_rc_ctrl->W_angle+FOLLOW_DEADBAND) / 180.0f * PI;
			else if (target_value_set->chassis_rc_ctrl->W_angle>FOLLOW_DEADBAND) 
				target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle + (target_value_set->chassis_rc_ctrl->W_angle-FOLLOW_DEADBAND) / 180.0f * PI;
			else 
				target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle;
			target_value_set->chassis_posture_info.yaw_gyro_set = 0.0f;
		}
	}
	else
	{
	 	target_value_set->chassis_posture_info.yaw_angle_sett = target_value_set->chassis_posture_info.yaw_angle;
	 	target_value_set->chassis_posture_info.yaw_gyro_set = target_value_set->chassis_posture_info.yaw_gyro;
	}



	// --------- Side Angle Set ---------
	// if( target_value_set->mode.sport_mode         != NONE                 &&
	// 	target_value_set->mode.sport_mode         != ABNORMAL_MOVING_MODE && 
	// 	target_value_set->flag_info.suspend_flag_R   == ON_GROUND &&
	// 	target_value_set->flag_info.suspend_flag_L   == ON_GROUND )
	// {
	// 	if( target_value_set->mode.sport_mode == SIDE_MODE )
	// 		target_value_set->chassis_posture_info.roll_angle_set = (fp32)(5.0 / 180.0 * PI) * 50.0f;
	// 	// else if( target_value_set->chassis_rc_ctrl->side_flag == -1 )
	// 	// 	target_value_set->chassis_posture_info.roll_angle_set = -(fp32)(5.0 / 180.0 * PI) * 50.0f;
	// 	else	
	// 		target_value_set->chassis_posture_info.roll_angle_set = 0.0f;
	// }
	// else
		target_value_set->chassis_posture_info.roll_angle_set = 0.0f;

	// ---------------- Leg Angle Set ----------
	if( target_value_set->mode.jumping_stage == PREPARING_STAND_JUMPING )
	{
		target_value_set->chassis_posture_info.leg_angle_L_set = -IDEAL_PREPARING_STAND_JUMPING_ANGLE;
		target_value_set->chassis_posture_info.leg_angle_R_set = -IDEAL_PREPARING_STAND_JUMPING_ANGLE;
	}
	else
	{
		target_value_set->chassis_posture_info.leg_angle_L_set = 0.0f;
		target_value_set->chassis_posture_info.leg_angle_R_set = 0.0f;
	}
	

	// ----------------- Chassis High Mode Update ----------------- 
	if( target_value_set->mode.sport_mode == ABNORMAL_MOVING_MODE || target_value_set->mode.sport_mode == TK_MODE)
		target_value_set->mode.chassis_high_mode = SIT_MODE;
	else if( target_value_set->mode.chassis_balancing_mode == FOOT_LAUNCHING )
		target_value_set->mode.chassis_high_mode = SIT_MODE;
	else if( target_value_set->mode.chassis_balancing_mode == JOINT_LAUNCHING )
		target_value_set->mode.chassis_high_mode = NORMAL_MODE;
	else if( target_value_set->mode.chassis_balancing_mode == JOINT_REDUCING )
		target_value_set->mode.chassis_high_mode = CHANGING_HIGH;
	else if( target_value_set->chassis_rc_ctrl->sit_flag )
		target_value_set->mode.chassis_high_mode = SIT_MODE;
	else if( target_value_set->chassis_rc_ctrl->high_flag )
		target_value_set->mode.chassis_high_mode = HIGH_MODE;
	else if( target_value_set->mode.jumping_stage == CONSTACTING_LEGS )
		target_value_set->mode.chassis_high_mode = SIT_MODE;
	else if( target_value_set->mode.jumping_stage == EXTENDING_LEGS )
		target_value_set->mode.chassis_high_mode = HIGH_MODE;
	else if( target_value_set->mode.jumping_stage == CONSTACTING_LEGS_2 )
		target_value_set->mode.chassis_high_mode = SIT_MODE;
	else if( target_value_set->mode.jumping_stage == PREPARING_LANDING )
		target_value_set->mode.chassis_high_mode = NORMAL_MODE;
	else
		target_value_set->mode.chassis_high_mode = NORMAL_MODE;

	// --------- Leg Length Set --------- 
	if( target_value_set->mode.chassis_high_mode == SIT_MODE )
		target_value_set->chassis_posture_info.ideal_high = SIT_HIGH;
	else if( target_value_set->mode.chassis_high_mode == NORMAL_MODE )
		target_value_set->chassis_posture_info.ideal_high = NORMAL_HIGH + High_Offset - Moving_High_Offset;
	else if( target_value_set->mode.chassis_high_mode == HIGH_MODE )
		target_value_set->chassis_posture_info.ideal_high = HIGH_HIGH + High_Offset - Moving_High_Offset;
	else if( target_value_set->mode.chassis_high_mode == CHANGING_HIGH )
	{
		reduce_high = reduce_high * debug_1;
		target_value_set->chassis_posture_info.ideal_high = min(reduce_high,target_value_set->chassis_posture_info.ideal_high);
			// target_value_set->chassis_posture_info.ideal_high = 
			// 	debug_1 * target_value_set->chassis_posture_info.ideal_high;
	}

	if( target_value_set->mode.sport_mode == ABNORMAL_MOVING_MODE ||
		( target_value_set->flag_info.suspend_flag_L == 1 && target_value_set->flag_info.suspend_flag_R == 1 ) ||
		target_value_set->chassis_posture_info.ideal_high == SIT_HIGH ||
		target_value_set->mode.chassis_balancing_mode == JOINT_REDUCING )
	{
		target_value_set->chassis_posture_info.leg_length_L_set = target_value_set->chassis_posture_info.ideal_high;
		target_value_set->chassis_posture_info.leg_length_R_set = target_value_set->chassis_posture_info.ideal_high;
	}
	else
	{
		target_value_set->chassis_posture_info.foot_roll_angle = 
			target_value_set->chassis_posture_info.roll_angle +
			atan(( target_value_set->chassis_posture_info.leg_length_L - target_value_set->chassis_posture_info.leg_length_R ) / 0.50f );
		
		target_value_set->chassis_posture_info.leg_length_L_set = 
			target_value_set->chassis_posture_info .ideal_high 
			+ 0.25f * arm_sin_f32( target_value_set->chassis_posture_info .foot_roll_angle ) / arm_cos_f32( target_value_set->chassis_posture_info .foot_roll_angle );
		target_value_set->chassis_posture_info.leg_length_R_set = 
			target_value_set->chassis_posture_info .ideal_high
			- 0.25f * arm_sin_f32( target_value_set->chassis_posture_info .foot_roll_angle ) / arm_cos_f32( target_value_set->chassis_posture_info .foot_roll_angle );
	}
		
}

void Chassis_Torque_Calculation(chassis_control_t *bl_ctrl)
{

	LQR_Data_Update(bl_ctrl);
	// -----Roll Balance-----
	if( bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1 ||
		bl_ctrl->mode.chassis_high_mode == SIT_MODE )
	{
		bl_ctrl->torque_info.joint_roll_torque_R = 0.0f;
		bl_ctrl->torque_info.joint_roll_torque_L = 0.0f;
	}
	else
	{
		if (bl_ctrl->chassis_posture_info.roll_angle < -roll_angle_deadband) {
			rollP = roll_PD[0] * (bl_ctrl->chassis_posture_info.roll_angle_set - (bl_ctrl->chassis_posture_info.roll_angle+roll_angle_deadband));
		} 
		else if (bl_ctrl->chassis_posture_info.roll_angle > roll_angle_deadband){
			rollP = roll_PD[0] * (bl_ctrl->chassis_posture_info.roll_angle_set - (bl_ctrl->chassis_posture_info.roll_angle-roll_angle_deadband));
		}
		else {
			rollP = 0.0f;
		}

		if (bl_ctrl->chassis_posture_info.roll_gyro < -roll_gyro_deadband) {
			rollD = roll_PD[1] * - (bl_ctrl->chassis_posture_info.roll_gyro + roll_gyro_deadband);
		} 
		else if (bl_ctrl->chassis_posture_info.roll_gyro > roll_gyro_deadband){
			rollD = roll_PD[1] * - (bl_ctrl->chassis_posture_info.roll_gyro - roll_gyro_deadband);
		}
		else {
			rollD = 0.0f;
		}
		bl_ctrl->torque_info.joint_roll_torque_R = rollP + rollD;
		bl_ctrl->torque_info.joint_roll_torque_L = -bl_ctrl->torque_info.joint_roll_torque_R;
	}
	

	// -----During turns: prevent displacement of two legs----- 
	bl_ctrl->torque_info.joint_prevent_splits_torque_L = 
		coordinate_PD[0] * (bl_ctrl->chassis_posture_info.leg_angle_L - bl_ctrl->chassis_posture_info.leg_angle_R)
		+ coordinate_PD[1] * (bl_ctrl->chassis_posture_info.leg_gyro_L - bl_ctrl->chassis_posture_info.leg_gyro_R);

	bl_ctrl->torque_info.joint_prevent_splits_torque_R = -bl_ctrl->torque_info.joint_prevent_splits_torque_L;

	if( bl_ctrl->mode.jumping_stage == EXTENDING_LEGS ||
		bl_ctrl->mode.jumping_stage == CONSTACTING_LEGS_2 )
	{
		bl_ctrl->torque_info.joint_stand_torque_L = 
			+ jump_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L ) 
			+ jump_stand_PD[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_L );
	
		bl_ctrl->torque_info.joint_stand_torque_R = 
			+ jump_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R ) 
			+ jump_stand_PD[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_R );
	}
	else if( bl_ctrl->mode.sport_mode == ABNORMAL_MOVING_MODE )
	{
		bl_ctrl->torque_info.joint_stand_torque_L = 
				+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L ) 
				+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_L );
		bl_ctrl->torque_info.joint_stand_torque_R = 
				+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R ) 
				+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_R );
	}
	else if( bl_ctrl->flag_info.suspend_flag_R == 1 || bl_ctrl->flag_info.suspend_flag_L == 1 )
	{
		if( bl_ctrl->flag_info.suspend_flag_L == 1 )
		{
			bl_ctrl->torque_info.joint_stand_torque_L = 
				+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L ) 
				+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_L );
		}
		else{
			bl_ctrl->torque_info.joint_stand_torque_L = 
				FEED_f
				+ stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L ) 
				+ stand_PD[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_L );
		}

		if( bl_ctrl->flag_info.suspend_flag_R == 1 )
		{
			bl_ctrl->torque_info.joint_stand_torque_R = 
				+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R ) 
				+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_R );
		} else {
			bl_ctrl->torque_info.joint_stand_torque_R = 
				FEED_f
				+ stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R ) 
				+ stand_PD[1] * ( 0 - bl_ctrl->chassis_posture_info.leg_dlength_R );
		}
	}
	else if( bl_ctrl->mode.chassis_balancing_mode == JOINT_REDUCING )
	{

		bl_ctrl->torque_info.joint_stand_torque_L = 
			+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L ) 
			+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_L );
	
		bl_ctrl->torque_info.joint_stand_torque_R = 
			+ suspend_stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R ) 
			+ suspend_stand_PD[1] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_dlength_R );
	}
	else
	{
		bl_ctrl->torque_info.joint_stand_torque_L = FEED_f;
		bl_ctrl->torque_info.joint_stand_torque_L += stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_L_set - bl_ctrl->chassis_posture_info.leg_length_L );
		if (bl_ctrl->chassis_posture_info.leg_dlength_L > leg_dlength_deadband){
			bl_ctrl->torque_info.joint_stand_torque_L += stand_PD[1] * ( 0 - (bl_ctrl->chassis_posture_info.leg_dlength_L-leg_dlength_deadband) );
		}
		else if (bl_ctrl->chassis_posture_info.leg_dlength_L < -leg_dlength_deadband){
			bl_ctrl->torque_info.joint_stand_torque_L += stand_PD[1] * ( 0 - (bl_ctrl->chassis_posture_info.leg_dlength_L+leg_dlength_deadband));
		}
		else {
			bl_ctrl->torque_info.joint_stand_torque_L += 0.0f;
		}
		bl_ctrl->torque_info.joint_stand_torque_R = FEED_f;
		bl_ctrl->torque_info.joint_stand_torque_R += stand_PD[0] * ( bl_ctrl->chassis_posture_info.leg_length_R_set - bl_ctrl->chassis_posture_info.leg_length_R );
		if (bl_ctrl->chassis_posture_info.leg_dlength_R > leg_dlength_deadband){
			bl_ctrl->torque_info.joint_stand_torque_R += stand_PD[1] * ( 0 - (bl_ctrl->chassis_posture_info.leg_dlength_R-leg_dlength_deadband) );
		}
		else if (bl_ctrl->chassis_posture_info.leg_dlength_R < -leg_dlength_deadband){
			bl_ctrl->torque_info.joint_stand_torque_R += stand_PD[1] * ( 0 - (bl_ctrl->chassis_posture_info.leg_dlength_R+leg_dlength_deadband));
		}
		else {
			bl_ctrl->torque_info.joint_stand_torque_R += 0.0f;
		}
		
	}
	// --------------------- joint motor calucaltion ---------------------


	// TODDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDO
	if( bl_ctrl->mode.jumping_stage == CONSTACTING_LEGS_2 )
	{
		bl_ctrl->torque_info.joint_balancing_torque_L = 
			-suspend_LQR[0][0] * ( 0 - bl_ctrl->chassis_posture_info.leg_angle_L ) 
			-suspend_LQR[0][1] * ( 0 - bl_ctrl->chassis_posture_info.leg_gyro_L );
		bl_ctrl->torque_info.joint_balancing_torque_R = 
			-suspend_LQR[0][0] * ( 0 - bl_ctrl->chassis_posture_info.leg_angle_R )
			-suspend_LQR[0][1] * ( 0 - bl_ctrl->chassis_posture_info.leg_gyro_R );

		bl_ctrl->torque_info.joint_moving_torque_L = 0.0f;
		bl_ctrl->torque_info.joint_moving_torque_R = 0.0f;
	}
	else 
	{
		if( bl_ctrl->mode.sport_mode == ABNORMAL_MOVING_MODE || bl_ctrl->mode.sport_mode == TK_MODE )
		{
			bl_ctrl->torque_info.joint_balancing_torque_L = 
			bl_ctrl->torque_info.joint_moving_torque_L    =
			bl_ctrl->torque_info.joint_balancing_torque_R =
			bl_ctrl->torque_info.joint_moving_torque_R    = 0;
		}
		else 
		{
			if( bl_ctrl->mode.chassis_high_mode == SIT_MODE ||
				bl_ctrl->mode.chassis_balancing_mode == JOINT_REDUCING )
			{
				bl_ctrl->torque_info.joint_balancing_torque_L = 
				bl_ctrl->torque_info.joint_moving_torque_L    =
				bl_ctrl->torque_info.joint_balancing_torque_R = 
				bl_ctrl->torque_info.joint_moving_torque_R    = 0;
			}
			else
			{
				
				bl_ctrl->torque_info.joint_balancing_torque_L = (
					-LQR[2][4] * ( bl_ctrl->chassis_posture_info.leg_angle_L_set - bl_ctrl->chassis_posture_info.leg_angle_L )
					-LQR[2][5] * (                  0.0f                         - bl_ctrl->chassis_posture_info.leg_gyro_L )
					-LQR[2][8] * ( bl_ctrl->chassis_posture_info.pitch_angle_set - bl_ctrl->chassis_posture_info.pitch_angle ) 
					-LQR[2][9] * ( bl_ctrl->chassis_posture_info.pitch_gyro_set  - bl_ctrl->chassis_posture_info.pitch_gyro ) );
				bl_ctrl->torque_info.joint_moving_torque_L    = ( 
					+LQR[2][0] * ( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K + NORMAL_MODE_WEIGHT_DISTANCE_OFFSET)
					+LQR[2][1] * ( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.foot_speed_K )
					+LQR[2][2] * ( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle )
					+LQR[2][3] * ( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
					);

				bl_ctrl->torque_info.joint_balancing_torque_R = (
					-LQR[3][6] * ( bl_ctrl->chassis_posture_info.leg_angle_R_set - bl_ctrl->chassis_posture_info.leg_angle_R ) 
					-LQR[3][7] * (                  0.0f                         - bl_ctrl->chassis_posture_info.leg_gyro_R )
					-LQR[3][8] * ( bl_ctrl->chassis_posture_info.pitch_angle_set - bl_ctrl->chassis_posture_info.pitch_angle ) 
					-LQR[3][9] * ( bl_ctrl->chassis_posture_info.pitch_gyro_set  - bl_ctrl->chassis_posture_info.pitch_gyro ) );
				bl_ctrl->torque_info.joint_moving_torque_R    = ( 
					+LQR[3][0] * ( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K + NORMAL_MODE_WEIGHT_DISTANCE_OFFSET)
					+LQR[3][1] * ( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.foot_speed_K )
					+LQR[3][2] * ( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle )
					+LQR[3][3] * ( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
					);
			}
		}
	}

	// --------------------- Foot motor LQR ---------------------
	if ( bl_ctrl->mode.sport_mode == TK_MODE ) {
		bl_ctrl->torque_info.foot_balancing_torque_L = 0.0f;
		bl_ctrl->torque_info.foot_balancing_torque_R = 0.0f;
		bl_ctrl->torque_info.foot_moving_torque_L = (int) (
			-TK_x_p*( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_speed_K ) 
			-TK_y_p*( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.yaw_angle )
			-TK_y_d*( 0.0f		                                      - bl_ctrl->chassis_posture_info.yaw_gyro )
		)* TORQ_K;
		bl_ctrl->torque_info.foot_moving_torque_R = (int)-(
			-TK_x_p*( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_speed_K ) 
			+TK_y_p*( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.yaw_angle )
			+TK_y_d*( 0.0f		                                      - bl_ctrl->chassis_posture_info.yaw_gyro )
		)* TORQ_K;
	}
	else {
		bl_ctrl->torque_info.foot_balancing_torque_L = (int) ( 
			-LQR[0][4]*( bl_ctrl->chassis_posture_info.leg_angle_L_set - bl_ctrl->chassis_posture_info.leg_angle_L) 
			-LQR[0][5]*(                  0.0f                         - bl_ctrl->chassis_posture_info.leg_gyro_L)
			+LQR[0][8]*( bl_ctrl->chassis_posture_info.pitch_angle_set - bl_ctrl->chassis_posture_info.pitch_angle) 
			+LQR[0][9]*( bl_ctrl->chassis_posture_info.pitch_gyro_set - bl_ctrl->chassis_posture_info.pitch_gyro) 
		)* TORQ_K;

		bl_ctrl->torque_info.foot_moving_torque_L = (int) ( 
			-LQR[0][0]*( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K + NORMAL_MODE_WEIGHT_DISTANCE_OFFSET)
			-LQR[0][1]*( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.foot_speed_K) 
			-LQR[0][2]*( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle)
			-LQR[0][3]*( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
		)* TORQ_K;

		bl_ctrl->torque_info.foot_balancing_torque_R = (int) -( 
			-LQR[1][6]*( bl_ctrl->chassis_posture_info.leg_angle_R_set  - bl_ctrl->chassis_posture_info.leg_angle_R) 
			-LQR[1][7]*(                  0.0f                         - bl_ctrl->chassis_posture_info.leg_gyro_R)  
			+LQR[1][8]*( bl_ctrl->chassis_posture_info.pitch_angle_set  - bl_ctrl->chassis_posture_info.pitch_angle) 
			+LQR[1][9]*( bl_ctrl->chassis_posture_info.pitch_gyro_set - bl_ctrl->chassis_posture_info.pitch_gyro) 
		)* TORQ_K;

		bl_ctrl->torque_info.foot_moving_torque_R = (int) -( 
			-LQR[1][0]*( bl_ctrl->chassis_posture_info.foot_distance_set - bl_ctrl->chassis_posture_info.foot_distance_K + NORMAL_MODE_WEIGHT_DISTANCE_OFFSET)
			-LQR[1][1]*( bl_ctrl->chassis_posture_info.foot_speed_set    - bl_ctrl->chassis_posture_info.foot_speed_K) 
			+LQR[1][2]*( bl_ctrl->chassis_posture_info.yaw_angle_sett    - bl_ctrl->chassis_posture_info.yaw_angle)
			+LQR[1][3]*( bl_ctrl->chassis_posture_info.yaw_gyro_set      - bl_ctrl->chassis_posture_info.yaw_gyro  )
		)* TORQ_K;
	}

	if( bl_ctrl->flag_info.suspend_flag_R == 1 )
	{
		bl_ctrl->torque_info.joint_balancing_torque_R = 
			-suspend_LQR[0][0] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_angle_R )
			-suspend_LQR[0][1] * ( 0 - bl_ctrl->chassis_posture_info.leg_gyro_R );

		bl_ctrl->torque_info.foot_moving_torque_R = 
			-suspend_foot_speed_p * ( 0.0f - bl_ctrl->foot_motor_R.speed );	

		bl_ctrl->torque_info.joint_moving_torque_R   = 0.0f;
		bl_ctrl->torque_info.foot_balancing_torque_R = 0.0f;
	}
	if( bl_ctrl->flag_info.suspend_flag_L == 1 )
	{
		bl_ctrl->torque_info.joint_balancing_torque_L = 
			-suspend_LQR[0][0] * ( 0.0f - bl_ctrl->chassis_posture_info.leg_angle_L ) 
			-suspend_LQR[0][1] * ( 0 - bl_ctrl->chassis_posture_info.leg_gyro_L );
		bl_ctrl->torque_info.foot_moving_torque_L = 
			+suspend_foot_speed_p * ( 0.0f - bl_ctrl->foot_motor_L.speed );

		bl_ctrl->torque_info.joint_moving_torque_L   = 0.0f;
		bl_ctrl->torque_info.foot_balancing_torque_L = 0.0f;
	}


	// LimitMax( bl_ctrl->torque_info.foot_moving_torque_L,  MAX_ACCL );
	// LimitMax( bl_ctrl->torque_info.foot_moving_torque_R,  MAX_ACCL );
	LimitMax( bl_ctrl->torque_info.joint_moving_torque_L, MAX_ACCL_JOINT );
	LimitMax( bl_ctrl->torque_info.joint_moving_torque_R, MAX_ACCL_JOINT );

}

void Chassis_Torque_Combine(chassis_control_t *bl_ctrl)
{
	bl_ctrl->mapping_info .J1_L = VMC_solve_J1(bl_ctrl->chassis_posture_info .leg_angle_L ,bl_ctrl->chassis_posture_info .leg_length_L );
	bl_ctrl->mapping_info .J2_L = VMC_solve_J2(bl_ctrl->chassis_posture_info .leg_angle_L ,bl_ctrl->chassis_posture_info .leg_length_L);  
	bl_ctrl->mapping_info .J3_L = VMC_solve_J3(bl_ctrl->chassis_posture_info .leg_angle_L ,bl_ctrl->chassis_posture_info .leg_length_L);
	bl_ctrl->mapping_info .J4_L = VMC_solve_J4(bl_ctrl->chassis_posture_info .leg_angle_L ,bl_ctrl->chassis_posture_info .leg_length_L );
	bl_ctrl->mapping_info .J1_R = VMC_solve_J1(bl_ctrl->chassis_posture_info .leg_angle_R ,bl_ctrl->chassis_posture_info .leg_length_R );
	bl_ctrl->mapping_info .J2_R = VMC_solve_J2(bl_ctrl->chassis_posture_info .leg_angle_R ,bl_ctrl->chassis_posture_info .leg_length_R);  
	bl_ctrl->mapping_info .J3_R = VMC_solve_J3(bl_ctrl->chassis_posture_info .leg_angle_R ,bl_ctrl->chassis_posture_info .leg_length_R);
	bl_ctrl->mapping_info .J4_R = VMC_solve_J4(bl_ctrl->chassis_posture_info .leg_angle_R ,bl_ctrl->chassis_posture_info .leg_length_R );

	bl_ctrl->torque_info.foot_horizontal_torque_L = 
		bl_ctrl->torque_info.foot_balancing_torque_L + bl_ctrl->torque_info.foot_moving_torque_L;
	bl_ctrl->torque_info.foot_horizontal_torque_R = 
		bl_ctrl->torque_info.foot_balancing_torque_R + bl_ctrl->torque_info.foot_moving_torque_R;

	bl_ctrl->foot_motor_L.torque_out = bl_ctrl->torque_info.foot_horizontal_torque_L;
	bl_ctrl->foot_motor_R.torque_out = bl_ctrl->torque_info.foot_horizontal_torque_R;

	LimitMax(bl_ctrl->foot_motor_L.torque_out,16383);
	LimitMax(bl_ctrl->foot_motor_R.torque_out,16383);

	bl_ctrl->torque_info.joint_horizontal_torque_L = 
		bl_ctrl->torque_info.joint_balancing_torque_L + bl_ctrl->torque_info.joint_moving_torque_L + bl_ctrl->torque_info.joint_prevent_splits_torque_L;
	bl_ctrl->torque_info.joint_horizontal_torque_R =
		bl_ctrl->torque_info.joint_balancing_torque_R + bl_ctrl->torque_info.joint_moving_torque_R + bl_ctrl->torque_info.joint_prevent_splits_torque_R;

	bl_ctrl->torque_info.joint_vertical_torque_L = 
		bl_ctrl->torque_info.joint_stand_torque_L + bl_ctrl->torque_info.joint_roll_torque_L;
	bl_ctrl->torque_info.joint_vertical_torque_R = 
		bl_ctrl->torque_info.joint_stand_torque_R + bl_ctrl->torque_info.joint_roll_torque_R;

	bl_ctrl->torque_info.joint_horizontal_torque_temp1_L = 
		(bl_ctrl->torque_info.joint_horizontal_torque_L) * bl_ctrl->mapping_info .J3_L ;
	bl_ctrl->torque_info.joint_horizontal_torque_temp2_L = 
		(bl_ctrl->torque_info.joint_horizontal_torque_L) * bl_ctrl->mapping_info .J4_L ;
	bl_ctrl->torque_info.joint_horizontal_torque_temp1_R = 
		(bl_ctrl->torque_info.joint_horizontal_torque_R) * bl_ctrl->mapping_info .J3_R ;
	bl_ctrl->torque_info.joint_horizontal_torque_temp2_R = 
		(bl_ctrl->torque_info.joint_horizontal_torque_R) * bl_ctrl->mapping_info .J4_R ;
	
	bl_ctrl->torque_info.joint_vertical_torque_temp1_L = 
		(bl_ctrl->torque_info.joint_vertical_torque_L) * bl_ctrl->mapping_info .J1_L;
	bl_ctrl->torque_info.joint_vertical_torque_temp2_L = 
		(bl_ctrl->torque_info.joint_vertical_torque_L) * bl_ctrl->mapping_info .J2_L;
	bl_ctrl->torque_info.joint_vertical_torque_temp1_R = 
		(bl_ctrl->torque_info.joint_vertical_torque_R) * bl_ctrl->mapping_info .J1_R;
	bl_ctrl->torque_info.joint_vertical_torque_temp2_R = 
		(bl_ctrl->torque_info.joint_vertical_torque_R) * bl_ctrl->mapping_info .J2_R;

	/****************************************/

	fp32 MAX_balance = 2000.0f;

	LimitMax(bl_ctrl->torque_info.joint_horizontal_torque_temp1_L,MAX_balance);
	LimitMax(bl_ctrl->torque_info.joint_horizontal_torque_temp2_L,MAX_balance);
	LimitMax(bl_ctrl->torque_info.joint_horizontal_torque_temp1_R,MAX_balance);
	LimitMax(bl_ctrl->torque_info.joint_horizontal_torque_temp2_R,MAX_balance);
	LimitMax(bl_ctrl->torque_info.joint_vertical_torque_temp1_L,15);
	LimitMax(bl_ctrl->torque_info.joint_vertical_torque_temp2_L,15);
	LimitMax(bl_ctrl->torque_info.joint_vertical_torque_temp1_R,15);
	LimitMax(bl_ctrl->torque_info.joint_vertical_torque_temp2_R,15);

	// 分配到四�?关节电机
	bl_ctrl->joint_motor_1.torque_out = + bl_ctrl->torque_info.joint_horizontal_torque_temp1_L + bl_ctrl->torque_info.joint_vertical_torque_temp1_L;
	bl_ctrl->joint_motor_2.torque_out = + bl_ctrl->torque_info.joint_horizontal_torque_temp2_L + bl_ctrl->torque_info.joint_vertical_torque_temp2_L;
	bl_ctrl->joint_motor_3.torque_out = - bl_ctrl->torque_info.joint_horizontal_torque_temp1_R + bl_ctrl->torque_info.joint_vertical_torque_temp1_R;
	bl_ctrl->joint_motor_4.torque_out = - bl_ctrl->torque_info.joint_horizontal_torque_temp2_R + bl_ctrl->torque_info.joint_vertical_torque_temp2_R;

	if( ABS(bl_ctrl->joint_motor_1.motor_measure->ecd) <= MOTOR_POS_UPPER_BOUND )
	{ 
		bl_ctrl->joint_motor_1.max_torque = LIMITED_TORQUE;
		bl_ctrl->joint_motor_1.min_torque = -1.0f * UNLIMITED_TORQUE;
	}
	else if( ABS(bl_ctrl->joint_motor_1.motor_measure->ecd) >= MOTOR_POS_LOWER_BOUND )
	{
		bl_ctrl->joint_motor_1.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_1.min_torque = -1.0f * LIMITED_TORQUE;
	}
	else
	{
		bl_ctrl->joint_motor_1.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_1.min_torque = -1.0f * UNLIMITED_TORQUE;
	}
	
	if( ABS(bl_ctrl->joint_motor_3.motor_measure->ecd) <= MOTOR_POS_UPPER_BOUND )
	{
		bl_ctrl->joint_motor_3.max_torque = LIMITED_TORQUE;
		bl_ctrl->joint_motor_3.min_torque = -1.0f * UNLIMITED_TORQUE;
	}
	else if( ABS(bl_ctrl->joint_motor_3.motor_measure->ecd) >= MOTOR_POS_LOWER_BOUND )
	{
		bl_ctrl->joint_motor_3.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_3.min_torque = -1.0f * LIMITED_TORQUE;
	}
	else
	{
		bl_ctrl->joint_motor_3.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_3.min_torque = -1.0f * UNLIMITED_TORQUE;
	}

	if( ABS(bl_ctrl->joint_motor_2.motor_measure->ecd) <= MOTOR_POS_UPPER_BOUND )
	{
		bl_ctrl->joint_motor_2.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_2.min_torque = -1.0f * LIMITED_TORQUE;
	}
	else if( ABS(bl_ctrl->joint_motor_2.motor_measure->ecd) >= MOTOR_POS_LOWER_BOUND )
	{
		bl_ctrl->joint_motor_2.max_torque = LIMITED_TORQUE;
		bl_ctrl->joint_motor_2.min_torque = -1.0f * UNLIMITED_TORQUE;
	}
	else
	{
		bl_ctrl->joint_motor_2.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_2.min_torque = -1.0f * UNLIMITED_TORQUE;
	}

	if( ABS(bl_ctrl->joint_motor_4.motor_measure->ecd) <= MOTOR_POS_UPPER_BOUND )
	{
		bl_ctrl->joint_motor_4.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_4.min_torque = -1.0f * LIMITED_TORQUE;
	}
	else if( ABS(bl_ctrl->joint_motor_4.motor_measure->ecd) >= MOTOR_POS_LOWER_BOUND )
	{
		bl_ctrl->joint_motor_4.max_torque = LIMITED_TORQUE;
		bl_ctrl->joint_motor_4.min_torque = -1.0f * UNLIMITED_TORQUE;
	}
	else
	{
		bl_ctrl->joint_motor_4.max_torque = UNLIMITED_TORQUE;
		bl_ctrl->joint_motor_4.min_torque = -1.0f * UNLIMITED_TORQUE;
	}

	LimitOutput( bl_ctrl->joint_motor_1.torque_out, bl_ctrl->joint_motor_1.min_torque, bl_ctrl->joint_motor_1.max_torque);
	LimitOutput( bl_ctrl->joint_motor_2.torque_out, bl_ctrl->joint_motor_2.min_torque, bl_ctrl->joint_motor_2.max_torque);
	LimitOutput( bl_ctrl->joint_motor_3.torque_out, bl_ctrl->joint_motor_3.min_torque, bl_ctrl->joint_motor_3.max_torque);
	LimitOutput( bl_ctrl->joint_motor_4.torque_out, bl_ctrl->joint_motor_4.min_torque, bl_ctrl->joint_motor_4.max_torque);
}

void Motor_CMD_Send(chassis_control_t *CMD_Send)
{
	if( CMD_Send->joint_motor_1.motor_mode == MOTOR_FORCE )
		CAN_HT_CMD( 0x01, CMD_Send->joint_motor_1.torque_out );
	else 
		CAN_HT_CMD( 0x01, 0.0 );
	if( CMD_Send->joint_motor_2.motor_mode == MOTOR_FORCE )
		CAN_HT_CMD( 0x02, CMD_Send->joint_motor_2.torque_out );
	else 
		CAN_HT_CMD( 0x02, 0.0 );


	if( CMD_Send->foot_motor_R.motor_mode != MOTOR_FORCE )
		CMD_Send->foot_motor_R .torque_out = 0.0f;
	if( CMD_Send->foot_motor_L.motor_mode != MOTOR_FORCE )
		CMD_Send->foot_motor_L .torque_out = 0.0f;
	CAN_BM_CONTROL_CMD( CMD_Send->foot_motor_L.torque_out, CMD_Send->foot_motor_R.torque_out );

	vTaskDelay(1);

	if( CMD_Send->joint_motor_3.motor_mode == MOTOR_FORCE )
		CAN_HT_CMD( 0x03, CMD_Send->joint_motor_3.torque_out );
	else 
		CAN_HT_CMD( 0x03, 0.0 );
	

	if( CMD_Send->joint_motor_4.motor_mode == MOTOR_FORCE )
		CAN_HT_CMD( 0x04, CMD_Send->joint_motor_4.torque_out );
	else 
		CAN_HT_CMD( 0x04, 0.0 );
}

void Joint_Motor_to_Init_Pos()
{
	int Init_Time = 0;
	while( Init_Time < 300 )
	{
		CAN_HT_CMD( 0x01, 1.0 );
		CAN_HT_CMD( 0x02, -1.0 );
		CAN_HT_CMD( 0x03, 1.0 );
		vTaskDelay(1);
		CAN_HT_CMD( 0x04, -1.0 );
		vTaskDelay(1);
		Init_Time++;
	}
}

void HT_Motor_zero_set(void)
{
	uint8_t tx_buff[8];
	for( int i = 0; i < 7; i++ )
		tx_buff[i] = 0xFF;
	tx_buff[7] = 0xfc;

	// ENABLE_CHASSIS
	CAN_CMD_HT_Enable( 0x01, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x02, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x03, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x04, tx_buff );
	vTaskDelay(50);
	// �?到限�?
	Joint_Motor_to_Init_Pos();
	// Set zero init point
	tx_buff[7] = 0xfe;

	CAN_CMD_HT_Enable( 0x01, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x02, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x03, tx_buff );
	vTaskDelay(50);
	CAN_CMD_HT_Enable( 0x04, tx_buff );

	vTaskDelay(50);

}

void Motor_Zero_CMD_Send(void)
{
	CAN_HT_CMD( 0x01, 0.0 );
	vTaskDelay(1);
	CAN_HT_CMD( 0x02, 0.0 );
	vTaskDelay(1);
	CAN_HT_CMD( 0x03, 0.0 );
	vTaskDelay(1);
	CAN_HT_CMD( 0x04, 0.0 );
	vTaskDelay(1);

	// CAN_BM_ENABLE_CMD();
	// vTaskDelay(1);

	// CAN_BM_MODE_SET_CMD();
	// vTaskDelay(2);

	// CAN_BM_CURRENT_MODE_CMD();
	// vTaskDelay(1);

}
