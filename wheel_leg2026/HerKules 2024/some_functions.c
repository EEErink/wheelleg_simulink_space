//             Front
//         | 1       4 |
//  (1)Left|           |Right(2)
//         | 2       3 |

#define HT_SLAVE_ID1        0x01
#define HT_SLAVE_ID2        0x02
#define HT_SLAVE_ID3        0x03
#define HT_SLAVE_ID4        0x04

#define CAN_BM_M1_ID 0x97
#define CAN_BM_M2_ID 0x98
#define BM_CAN hcan2

#define get_HT_motor_measure(ptr, data)                                    \
{                        \
	(ptr)->last_ecd = (ptr)->ecd;                                   \
	(ptr)->ecd =       uint_to_float((uint16_t)((data)[1] << 8 | (data)[2] ),P_MIN, P_MAX, 16);      \
	(ptr)->speed_rpm = uint_to_float((uint16_t)(data[3]<<4)|(data[4]>>4), V_MIN, V_MAX, 12);\
	(ptr)->real_torque = uint_to_float((uint16_t)(((data[4] & 0x0f) << 8) | (data)[5]), -18, +18, 12); \
}	

#define get_BM_motor_measure(ptr, data)                           	\
{                                                                   \
	(ptr)->last_ecd = (ptr)->ecd;                                   \
	(ptr)->ecd = (uint16_t)((data)[4] << 8 | (data)[5]);            \
	(ptr)->speed_rpm = (int16_t)((data)[0] << 8 | (data)[1]);      \
	(ptr)->given_current = (int16_t)((data)[2] << 8 | (data)[3]);  \
	(ptr)->error = (data)[6]; \
	(ptr)->mode =  (data)[7];                                   \
}

void CAN_BM_ENABLE_CMD(void)
{
	CAN_TxHeaderTypeDef	bm_tx_measure;
	uint8_t				bm_can_send_data[8];

	uint32_t send_mail_box;
	bm_tx_measure.StdId = 0x105; // 120
	bm_tx_measure.IDE = CAN_ID_STD;
	bm_tx_measure.RTR = CAN_RTR_DATA;
	bm_tx_measure.DLC = 0x08;
	bm_can_send_data[0] = 0x0A;
	bm_can_send_data[1] = 0x0A;
	bm_can_send_data[2] = 0x00;
	bm_can_send_data[3] = 0x00;
	bm_can_send_data[4] = 0x00;
	bm_can_send_data[5] = 0x00;
	bm_can_send_data[6] = 0x00;
	bm_can_send_data[7] = 0x00;

	HAL_CAN_AddTxMessage(&BM_CAN, &bm_tx_measure, bm_can_send_data, &send_mail_box);
}

void CAN_BM_MODE_SET_CMD(void)
{
	CAN_TxHeaderTypeDef	bm_tx_measure;
	uint8_t				bm_can_send_data[8];

	uint32_t send_mail_box;
	bm_tx_measure.StdId = 0x106; // 120
	bm_tx_measure.IDE = CAN_ID_STD;
	bm_tx_measure.RTR = CAN_RTR_DATA;
	bm_tx_measure.DLC = 0x08;
	bm_can_send_data[0] = 0x01;
	bm_can_send_data[1] = 0x01;
	bm_can_send_data[2] = 0x00;
	bm_can_send_data[3] = 0x00;
	bm_can_send_data[4] = 0x00;
	bm_can_send_data[5] = 0x00;
	bm_can_send_data[6] = 0x00;
	bm_can_send_data[7] = 0x00;

	HAL_CAN_AddTxMessage(&BM_CAN, &bm_tx_measure, bm_can_send_data, &send_mail_box);
}

void CAN_BM_CONTROL_CMD( int16_t motor1, int16_t motor2 )
{
	CAN_TxHeaderTypeDef	bm_tx_measure;
	uint8_t				bm_can_send_data[8];

	uint32_t send_mail_box;
	bm_tx_measure.StdId = 0x32; // 120
	bm_tx_measure.IDE = CAN_ID_STD;
	bm_tx_measure.RTR = CAN_RTR_DATA;
	bm_tx_measure.DLC = 0x08;
	bm_can_send_data[0] = motor1 >> 8;
	bm_can_send_data[1] = motor1;
	bm_can_send_data[2] = motor2 >> 8;
	bm_can_send_data[3] = motor2;
	bm_can_send_data[4] = 0x00;
	bm_can_send_data[5] = 0x00;
	bm_can_send_data[6] = 0x00;
	bm_can_send_data[7] = 0x00;

	HAL_CAN_AddTxMessage(&BM_CAN, &bm_tx_measure, bm_can_send_data, &send_mail_box);
}

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define P_MIN -95.5f// Radians
#define P_MAX 95.5f
#define V_MIN -45.0f// Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f// N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f// N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

void CAN_HT_CMD( uint8_t id, fp32 f_t )
{
    uint32_t canTxMailbox = CAN_TX_MAILBOX0;

    fp32 f_p = 0.0f, f_v = 0.0f, f_kp = 0.0f, f_kd = 0.0f;
    uint16_t p, v, kp, kd, t;
    uint8_t buf[8];
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12);
    
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;

    chassis_tx_message.StdId = id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;

		// while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
		if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET) 
		{canTxMailbox = CAN_TX_MAILBOX0;} 
		else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET) 
		{canTxMailbox = CAN_TX_MAILBOX1;} 
		else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET) 
		{canTxMailbox = CAN_TX_MAILBOX2;}
		
	if(HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, buf, (uint32_t *)&canTxMailbox)==HAL_OK){};
}
void CAN_CMD_HT_Enable(uint8_t id, uint8_t unterleib_motor_send_data[8] )
{
    uint32_t canTxMailbox= CAN_TX_MAILBOX0;

    chassis_tx_message.StdId = id;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;

	// 	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
		if ((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET) 
		{canTxMailbox = CAN_TX_MAILBOX0;} 
		else if ((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET) 
		{canTxMailbox = CAN_TX_MAILBOX1;} 
		else if ((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET) 
		{canTxMailbox = CAN_TX_MAILBOX2;}
		
	if(HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, unterleib_motor_send_data, (uint32_t *)&canTxMailbox)==HAL_OK){};
}

void Forward_kinematic_solution(chassis_control_t *feedback_update,
											fp32 Q1,fp32 S1,fp32 Q4,fp32 S4, uint8_t ce)
{
	fp32 dL0=0,L0=0,Q0=0,S0=0;
	fp32 xb,xd,yb,yd,Lbd,xc,yc;
	fp32 A0,B0,C0,Q2,Q3,S2,S3;
	fp32 vxb,vxd,vyb,vyd,vxc,vyc;
	fp32 cos_Q1,cos_Q4,sin_Q1,sin_Q4;
	fp32 sin_Q2,cos_Q2,sin_Q3,cos_Q3;
	fp32 axb,ayb,axd,ayd,a2,axc;
	/******************************/
	Q1 = PI + Q1;
	cos_Q1 = arm_cos_f32(Q1);
	sin_Q1 = arm_sin_f32(Q1);
	cos_Q4 = arm_cos_f32(Q4);
	sin_Q4 = arm_sin_f32(Q4);
	xb = -L5/2.0f + L1*cos_Q1;
	xd =  L5/2.0f + L4*cos_Q4;
	yb = L1*sin_Q1;
	yd = L4*sin_Q4;

	Lbd=(xd-xb)*(xd-xb)+(yd-yb)*(yd-yb);
	A0 = 2.0f*L2*(xd-xb);
	B0 = 2.0f*L2*(yd-yb);
	C0 = L2*L2+Lbd-L3*L3;
	Q2 = 2.0f *atan_tl((B0+Sqrt(A0*A0 + B0*B0 -C0*C0))/(A0+C0));
	
	xc = xb + arm_cos_f32(Q2)*L2;
	yc = yb + arm_sin_f32(Q2)*L2;

	L0=Sqrt( xc*xc + yc*yc );
	Q0 = atan(yc/xc);
	
	vxb = -S1*L1*sin_Q1;
	vyb = S1*L1*cos_Q1;
	vxd = -S4*L4*sin_Q4;
	vyd = S4*L4*cos_Q4;
	Q3 = atan_tl((yc-yd)/(xc-xd));
	S2 = ((vxd-vxb)*arm_cos_f32(Q3) + (vyd-vyb)*arm_sin_f32(Q3))/(L2*arm_sin_f32(Q3-Q2)); 
	S3 = ((vxd-vxb)*arm_cos_f32(Q2) + (vyd-vyb)*arm_sin_f32(Q2))/(L2*arm_sin_f32(Q3-Q2)); 
	vxc = vxb - S2*L2*arm_sin_f32(Q2);
  	vyc = vyb + S2*L2*arm_cos_f32(Q2);
	S0 = 3*(-arm_sin_f32(ABS(Q0))*vxc-arm_cos_f32(Q0)*vyc);

	if( Q0 < 0 )
		Q0 += PI;
	/*******************************/
	if (ce)
	{
		feedback_update->chassis_posture_info .leg_length_L = L0;
		feedback_update->chassis_posture_info .leg_angle_L  = Q0;
		// feedback_update->chassis_posture_info .leg_gyro_L   		= S0;
	}
	else 
	{
		feedback_update->chassis_posture_info .leg_length_R = L0;
		feedback_update->chassis_posture_info .leg_angle_R  = Q0;
		// feedback_update->chassis_posture_info .leg_gyro_R   		= S0;
	}
}

void Supportive_Force_Cal( chassis_control_t *Suspend_Detect, fp32 Q1, fp32 Q4, fp32 ce )
{
	fp32 dL0=0,L0=0,Q0=0,S0=0;
	fp32 xb,xd,yb,yd,Lbd,xc,yc;
	fp32 A0,B0,C0,Q2,Q3,S2,S3;
	fp32 vxb,vxd,vyb,vyd,vxc,vyc;
	fp32 cos_Q1,cos_Q4,sin_Q1,sin_Q4;
	fp32 sin_Q2,cos_Q2,sin_Q3,cos_Q3;
	fp32 axb,ayb,axd,ayd,a2,axc;
	/******************************/
	Q1 = PI + Q1;
	cos_Q1 = arm_cos_f32(Q1);
	sin_Q1 = arm_sin_f32(Q1);
	cos_Q4 = arm_cos_f32(Q4);
	sin_Q4 = arm_sin_f32(Q4);
	xb = -L5/2.0f + L1*cos_Q1;
	xd =  L5/2.0f + L4*cos_Q4;
	yb = L1*sin_Q1;
	yd = L4*sin_Q4;

	Lbd=sqrt( (xd-xb)*(xd-xb)+(yd-yb)*(yd-yb) );
	A0 = 2.0f*L2*(xd-xb);
	B0 = 2.0f*L2*(yd-yb);
	C0 = L2*L2+Lbd*Lbd-L3*L3;
	Q2 = 2.0f *atan_tl((B0+Sqrt(A0*A0 + B0*B0 -C0*C0))/(A0+C0));
	xc = xb + arm_cos_f32(Q2)*L2;
	yc = yb + arm_sin_f32(Q2)*L2;

	L0 = Sqrt( xc*xc + yc*yc );
	Q0 = atan_tl(yc/xc);
	if( Q0 < 0 )
		Q0 += PI;
	
	Q3  = atan_tl((yc-yd)/(xc-xd))+PI;

	

	fp32 J1 = ( L1 * arm_sin_f32(Q0 - Q3) * arm_sin_f32( Q1 - Q2 ) ) / (arm_sin_f32( Q3 - Q2 ));
	fp32 J2 = ( L1 * arm_cos_f32(Q0 - Q3) * arm_sin_f32( Q1 - Q2 ) ) / (arm_sin_f32( Q3 - Q2 ) * L0);
	fp32 J3 = ( L4 * arm_sin_f32(Q0 - Q2) * arm_sin_f32( Q3 - Q4 ) ) / (arm_sin_f32( Q3 - Q2 ));
	fp32 J4 = ( L4 * arm_cos_f32(Q0 - Q2) * arm_sin_f32( Q3 - Q4 ) ) / (arm_sin_f32( Q3 - Q2 ) * L0);
	
	
	fp32 dett = J1 * J4 - J2 * J3;
	fp32 Inv_J1 =  J4 / dett;
	fp32 Inv_J2 = -J2 / dett;
	fp32 Inv_J3 = -J3 / dett;
	fp32 Inv_J4 =  J1 / dett;

	ddebug = yc;

	if( ce == 1.0f )
	{
		Suspend_Detect->chassis_posture_info.supportive_force_L = 
			Inv_J1 * Suspend_Detect->joint_motor_1.torque_get + 
			Inv_J2 * Suspend_Detect->joint_motor_2.torque_get;
		// if( Suspend_Detect->chassis_posture_info.supportive_force_L < 0.0f )
		// 	Suspend_Detect->chassis_posture_info.supportive_force_L += 40.0f;
	}
	else	
	{
		Suspend_Detect->chassis_posture_info.supportive_force_R = 
			Inv_J1 * Suspend_Detect->joint_motor_4.torque_get + 
			Inv_J2 * Suspend_Detect->joint_motor_3.torque_get;
		Suspend_Detect->chassis_posture_info.supportive_force_R *= -1.0f;
		// if( Suspend_Detect->chassis_posture_info.supportive_force_R < 0.0f )
		// 	Suspend_Detect->chassis_posture_info.supportive_force_R += 40.0f;
	}

}