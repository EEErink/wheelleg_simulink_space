#ifndef _CHASSIS_TASK
#define _CHASSIS_TASK

#include "main.h"
#include "struct_typedef.h"
#include "pid.h"
#include "bsp_can.h"
// ------------- Limit info ------------- 
#define MAX_ACCL 13000.0f
#define MAX_ACCL_JOINT 15.0f
#define MAX_FOOT_OUTPUT 2048

//  ------------- Target value info ------------- 
#define SIT_MODE_HEIGHT_SET            0.18f
#define NORMAL_MODE_HEIGHT_SET         0.10f
#define HIGH_MODE_HEIGHT_SET           0.27f
#define EXTREMELY_HIGH_MODE_HEIGHT_SET 0.30f

// ------------- Mech info ------------- 
#define L1 0.15f
#define L2 0.25f
#define L3 0.25f
#define L4 0.15f
#define L5 0.1f

#define WHEEL_PERIMETER 0.56547
#define WHEEL_RADIUS 0.09f
#define LEG_OFFSET       0.3790855135f // 水平位置到上限位的夹角
#define LOWER_SUPPORT_FORCE_FOR_JUMP 5.0f
#define LOWER_SUPPORT_FORCE 0.0f
#define MOVE_LOWER_BOUND 0.3f
#define EXIT_PITCH_ANGLE 0.1f
#define DANGER_PITCH_ANGLE 0.25f

#define FEED_f 7.5f
#define FEED_f_1 3.5f

#define NORMAL_MODE_WEIGHT_DISTANCE_OFFSET -0.0f

#define MOTOR_POS_UPPER_BOUND 0.05f
#define MOTOR_POS_LOWER_BOUND 1.4f
#define LIMITED_TORQUE 0.5f
#define UNLIMITED_TORQUE 200.0f

// ------------- Time info ------------- 
#define CHASSIS_TASK_INIT_TIME  500
#define TASK_RUN_TIME           0.002f

// ------------- Transfer info ------------- 
#define MOTOR_ECD_TO_RAD     0.00019174779      // 2*PI / 32767
#define HALF_ECD_RANGE                14383
#define HALF_POSITION_RANGE 3.0f
// #define CC                          0.00512f 
// #define CC                          1/494.0f
#define TORQ_K                      494.483818182
// ------------- Math info ------------- 
#define PI2					  6.28318530717959f
#define PI					  3.14159265358979f
#define PI_2				  1.57079632679489f
#define PI_4				  0.78539816339744f



typedef enum
{
    ENABLE_CHASSIS = 0,
    DISABLE_CHASSIS,
} chassis_mode_e;

typedef enum
{
    NO_FORCE,
    FOOT_LAUNCHING,
    JOINT_LAUNCHING,
    BALANCING_READY,
    JOINT_REDUCING,
} chassis_balancing_mode_e;

typedef enum
{
    NONE,
    NORMAL_MOVING_MODE,
    ABNORMAL_MOVING_MODE,
    JUMPING_MODE,
    CAP_MODE,
    FLY_MODE,
    TK_MODE,
} sport_mode_e;

typedef enum
{
    READY_TO_JUMP,
    PREPARING_LANDING,
    PREPARING_STAND_JUMPING,
    CONSTACTING_LEGS,
    EXTENDING_LEGS,
    CONSTACTING_LEGS_2,
    FINISHED,
} jumping_stage_e;

typedef enum
{
    NOT_DEFINE,
    STANDING_JUMP,
    MOVING_JUMP,
} jumping_mode_e;

typedef enum
{
    SIT_MODE = 0,
    NORMAL_MODE,
    HIGH_MODE,
    EXTREMELY_HIGH_MODE,
    CHANGING_HIGH,
} chassis_high_mode_e;

typedef enum
{
    MOTOR_NO_FORCE = 0,
    MOTOR_FORCE,
} chassis_motor_mode_e;

typedef enum
{
	ON_GROUND = 0,
	OFF_GROUND = 1,
} suspend_flag_e;

typedef struct
{
    chassis_mode_e           chassis_mode, last_chassis_mode;
    chassis_balancing_mode_e chassis_balancing_mode, last_chassis_balancing_mode;
    sport_mode_e             sport_mode, last_sport_mode; 
    
    jumping_mode_e           jumping_mode, last_jumping_mode;
    jumping_stage_e          jumping_stage, last_jumping_stage;

    chassis_high_mode_e       chassis_high_mode, last_chassis_high_mode;

} mode_t;

typedef struct
{
    const fp32 *chassis_INS_angle_point;
  	const fp32 *chassis_INS_gyro_point;
    const fp32 *chassis_INS_accel_point;
    fp32 yaw_angle, pitch_angle, roll_angle;
    fp32 yaw_gyro, pitch_gyro, roll_gyro;
    fp32 yaw_accel, pitch_accel, roll_accel;

    fp32 yaw_angle_sett, pitch_angle_set, roll_angle_set;
    fp32 yaw_gyro_set, pitch_gyro_set, roll_gyro_set;

    fp32 ideal_high;
    fp32 leg_length_L, last_leg_length_L, leg_length_L_set;
    fp32 leg_length_R, last_leg_length_R, leg_length_R_set;
    fp32 leg_dlength_L;
    fp32 leg_dlength_R;

    fp32 foot_roll_angle;
    fp32 leg_angle_L, last_leg_angle_L, leg_angle_L_set;
    fp32 leg_angle_R, last_leg_angle_R, leg_angle_R_set;
    fp32 leg_gyro_L, leg_gyro_R;

    fp32 foot_distance, foot_distance_K, foot_distance_set;
    fp32 foot_speed, foot_speed_K, foot_speed_set;

    fp32 supportive_force_L, supportive_force_R;

} chassis_posture_info_t;

typedef struct
{
    // -------- horizontal force -------- 
    fp32 joint_balancing_torque_L, joint_balancing_torque_R;
    fp32 foot_balancing_torque_L,  foot_balancing_torque_R;
	
	fp32 foot_moving_torque_L,  foot_moving_torque_R;
    fp32 joint_moving_torque_L, joint_moving_torque_R;

    fp32 joint_prevent_splits_torque_L, joint_prevent_splits_torque_R;

    fp32 joint_horizontal_torque_L, joint_horizontal_torque_R;
    fp32 foot_horizontal_torque_L,  foot_horizontal_torque_R;

    fp32 joint_horizontal_torque_temp1_R, joint_horizontal_torque_temp2_R;
    fp32 joint_horizontal_torque_temp1_L, joint_horizontal_torque_temp2_L;

    fp32 yaw_torque;

    // -------- vertical force -------- 
    fp32 joint_roll_torque_L,  joint_roll_torque_R;
    fp32 joint_stand_torque_L, joint_stand_torque_R;

    fp32 joint_vertical_torque_L,      joint_vertical_torque_R;
    fp32 joint_real_vertical_torque_L, joint_real_vertical_torque_R;

    fp32 joint_vertical_torque_temp1_R, joint_vertical_torque_temp2_R;
    fp32 joint_vertical_torque_temp1_L, joint_vertical_torque_temp2_L;

} torque_info_t;

typedef struct 
{
    fp32 J1_L,J2_L;
	fp32 J3_L,J4_L;
	fp32 J1_R,J2_R;
	fp32 J3_R,J4_R;
	fp32 invJ1_L,invJ2_L;
	fp32 invJ3_L,invJ4_L;
	fp32 invJ1_R,invJ2_R;
	fp32 invJ3_R,invJ4_R;

} mapping_info_t;

typedef struct
{
    const HT_motor_measure_t *motor_measure;
    chassis_motor_mode_e motor_mode, last_motor_mode;

    bool_t offline_flag;

    fp32 position;
    fp32 init_position;
    fp32 position_offset;

    fp32 velocity;
    fp32 torque_out, torque_get;
    fp32 max_torque, min_torque;
} joint_motor_t;

typedef struct
{
    motor_measure_t motor_measure;
    chassis_motor_mode_e motor_mode, last_motor_mode;

    bool_t offline_flag;

    fp32 distance, distance_offset, last_position, position, turns;
    fp32 speed;
    fp32 torque_out, torque_get;

} foot_motor_t;

typedef struct
{
    bool_t init_flag;
	suspend_flag_e suspend_flag_L, last_suspend_flag_L;
    suspend_flag_e suspend_flag_R, last_suspend_flag_R;
    bool_t Ignore_Off_Ground;
    bool_t abnormal_flag;
    bool_t static_flag, last_static_flag;
    bool_t moving_flag, last_moving_flag;
    bool_t rotation_flag;
    bool_t controlling_flag;
    bool_t set_pos_after_moving;
    bool_t overpower_warning_flag;
    bool_t last_overpower_warning_flag;
    bool_t stablize_high_flag;
    bool_t last_stablize_high_flag;
} flag_info_t;

typedef struct 
{
	pid_type_def buffer_control_pid;
	pid_type_def cap_pid;
	pid_type_def scale_down_pid;
} pid_info_t;

typedef struct
{
    mode_t                  mode;
    chassis_posture_info_t  chassis_posture_info;
    torque_info_t           torque_info;
    mapping_info_t          mapping_info;
    flag_info_t             flag_info;
	pid_info_t              pid_info;
    const Gimbal_ctrl_t     *chassis_rc_ctrl;

    joint_motor_t joint_motor_1, joint_motor_2, joint_motor_3, joint_motor_4;
    foot_motor_t foot_motor_L, foot_motor_R;

}chassis_control_t;

enum Chassis_Mode
{
  Chassis_No_Force = 0,
  Follow_Gimbal,
  Follow_Gimbal_90Deg,
  No_Follow,
  Rotate,
//   TK_MODE,
};

extern enum Chassis_Mode chassis_mode;
extern chassis_control_t chassis_control;
extern fp32 roll_PD[2];

#endif
