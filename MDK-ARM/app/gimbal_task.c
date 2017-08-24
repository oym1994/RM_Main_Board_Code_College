/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       gimbal_task.c
	* @brief      basic gimbal control and autoshoot task
	* @update	        
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   Richard.luo 
  * @verbatim
	* yaw axis  : angle feedback is single axis gyro, palstance feedback is mpu6500 gyro
	* pitch axis: angle feedback is encoder, palstance feedback is mpu6500 gyro
	********************************(C) COPYRIGHT 2017 DJI************************
	*/

#include "gimbal_task.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "calibrate.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "error_task.h"
#include "judge_sys.h"
#include "kb.h"
#include "mpu.h"
#include "mytype.h"
#include "pid.h"
#include "stm32f4xx_hal.h"
#include "sys.h"
#include <math.h>


/* for debuge */
int pit_p_r, pit_p_f, pit_s_r, pit_s_f;
int yaw_p_r, yaw_p_f, yaw_s_r, yaw_s_f;

gimbal_yaw_t gYaw;

/* gimbal pid parameter */
float yaw_angle_ref     = 0;
float pit_angle_ref     = 0; 
float yaw_angle_fdb     = 0;
float pit_angle_fdb     = 0; 
float yaw_speed_ref     = 0;
float pit_speed_ref     = 0;
float yaw_speed_fdb     = 0;
float pit_speed_fdb     = 0;
float pit_kp = 0;
float yaw_kp = -0.53;

/* read from flash */
int   pit_center_offset = 0; 
int   yaw_center_offset = 0;

/* gimbal relative position param */
s16   pit_relative_pos;
s16   yaw_relative_pos;   //unit : encoder
float pit_relative_angle;
float yaw_relative_angle; //unit : degree
		
/* shoot task relevant param */
int   last_sw1;
int   auto_shoot_cmd = 0;
int   fric_wheel_run = 0;
int   trigger_spd_ref, trigger_pos_ref; 
int   trigger_dir = 1;

/* imu relevant param */
float imu_tmp;
float imu_tmp_ref = 40; //keep imu in 40 degree


/* whether chassis follow gimbal when autoshoot */
int auto_chassis_follow_gim = 1;

/* automatic aim param */
int AimDirectionBiasYaw = -30;
int AimDirectionBiasPit = 0;
int CompensationY = 65;

/* vision data */
int YawBias = 0;
int PitBias = 0;
float PitchTarget = 0; 
float YawTarget = 0;
float LastPitchTarget = 0; 
float LastYawTarget = 0;
float RatioPitch = 12.20;
float RatioYaw = 16.4;
uint32_t no_vision_time;
double pnp_dis;

/**
  * @brief     get relative position angle to center
  * @param[in] raw_ecd: gimbal motor encoder raw angle
	* @param[in] center_offset: read GimbalCaliData from chip flash
	* @retval    relative angle, unit is degree.
  * @attention you should read center offset data in chip flash, 
	*            as: GimbalCaliData.GimbalPit/Yaw Offset
  */
s16 get_relative_pos(s16 raw_ecd, s16 center_offset)
{
    s16 tmp = 0;
    if (center_offset >= 4096)
    {
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else
    {
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }
    return tmp;
}
/**
  * @brief     send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param     current value corresponding motor(yaw/pitch/trigger)
  */
void can_send_gimbal_iq(s16 yaw_iq, s16 pit_iq, s16 trigger_iq)
{

    GIMBAL_CAN.pTxMsg->StdId   = 0x1ff;
    GIMBAL_CAN.pTxMsg->IDE     = CAN_ID_STD;
    GIMBAL_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
    GIMBAL_CAN.pTxMsg->DLC     = 8;
    GIMBAL_CAN.pTxMsg->Data[0] = yaw_iq >> 8;
    GIMBAL_CAN.pTxMsg->Data[1] = yaw_iq;
    GIMBAL_CAN.pTxMsg->Data[2] = pit_iq >> 8;
    GIMBAL_CAN.pTxMsg->Data[3] = pit_iq;
    GIMBAL_CAN.pTxMsg->Data[4] = trigger_iq >> 8;
    GIMBAL_CAN.pTxMsg->Data[5] = trigger_iq;
    GIMBAL_CAN.pTxMsg->Data[6] = 0;
    GIMBAL_CAN.pTxMsg->Data[7] = 0;
    HAL_CAN_Transmit(&GIMBAL_CAN, 1000);
}
/**
  * @brief     send particular message to calibrate 6025/6623 gimbal motor esc
  * @usage     only need calibrate motor when replace a new ESC
  */
void can_cali_6025()
{

    GIMBAL_CAN.pTxMsg->StdId   = 0x3f0;
    GIMBAL_CAN.pTxMsg->IDE     = CAN_ID_STD;
    GIMBAL_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
    GIMBAL_CAN.pTxMsg->DLC     = 8;
    GIMBAL_CAN.pTxMsg->Data[0] = 'c';
    GIMBAL_CAN.pTxMsg->Data[1] = 0;
    GIMBAL_CAN.pTxMsg->Data[2] = 0 >> 8;
    GIMBAL_CAN.pTxMsg->Data[3] = 0;
    GIMBAL_CAN.pTxMsg->Data[4] = 0 >> 8;
    GIMBAL_CAN.pTxMsg->Data[5] = 0;
    GIMBAL_CAN.pTxMsg->Data[6] = 0;
    GIMBAL_CAN.pTxMsg->Data[7] = 0;
    HAL_CAN_Transmit(&GIMBAL_CAN, 1000);
}
void can_cali_6623()
{

    GIMBAL_CAN.pTxMsg->StdId   = 0x1ff;
    GIMBAL_CAN.pTxMsg->IDE     = CAN_ID_STD;
    GIMBAL_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
    GIMBAL_CAN.pTxMsg->DLC     = 8;
    GIMBAL_CAN.pTxMsg->Data[0] = 0;
    GIMBAL_CAN.pTxMsg->Data[1] = 0;
    GIMBAL_CAN.pTxMsg->Data[2] = 0 >> 8;
    GIMBAL_CAN.pTxMsg->Data[3] = 0;
    GIMBAL_CAN.pTxMsg->Data[4] = 0 >> 8;
    GIMBAL_CAN.pTxMsg->Data[5] = 0;
    GIMBAL_CAN.pTxMsg->Data[6] = 4;
    GIMBAL_CAN.pTxMsg->Data[7] = 0;
    HAL_CAN_Transmit(&GIMBAL_CAN, 1000);
}








void auto_shoot_pid_init(void)
{

	PID_struct_init(&pid_pit, POSITION_PID, 2000, 50,
									50.0f, 0.2, 0.0f); //
	PID_struct_init(&pid_pit_speed, POSITION_PID, 5000, 2000,
									40.0f, 0.6, 0.0f);
    
    PID_struct_init(&pid_yaw, POSITION_PID, 2000, 5,
									30, 0.2, 0.0); //
	PID_struct_init(&pid_yaw_speed, POSITION_PID, 8000, 2000,
									60, 0.1, 0.0);
}
void normal_pid_init(void)
{
	                                       //maxout   intergral_limit	
	PID_struct_init(&pid_pit, POSITION_PID, 4000, 1000,
									200.0f, 0.5, 0); //
	PID_struct_init(&pid_pit_speed, POSITION_PID, 4000, 1000,
									20.0f, 0.2, 0.8f);

	PID_struct_init(&pid_yaw, POSITION_PID, 9000, 400,
									180, 0.4, 150); //
	PID_struct_init(&pid_yaw_speed, POSITION_PID, 8000, 800,
									20, 0.3, 20);
}
void gimbal_pid_init(void)
{
  normal_pid_init();	

	PID_struct_init(&pid_trigger, POSITION_PID, 10000, 2000, //position
									15.0f, 0, 10);
	PID_struct_init(&pid_trigger_speed, POSITION_PID, 7000, 4000, //vw
									1.5f, 0.3f, 5);

	PID_struct_init(&pid_imu_tmp, POSITION_PID, 999, 999, //position
									180, 0.1f, 0);
}


void shoot_task()
{
	if (fric_wheel_run && auto_shoot_cmd)
		trigger_spd_ref = -4000;
		//trigger_pos_ref += trigger_MOTOR_SPEED * trigger_dir;
	else
		trigger_spd_ref = 0;

	if (gYaw.ctrl_mode == GIMBAL_AUTO_SHOOT && chassis.mode == CHASSIS_AUTO)
	{
		TIM12->CCR1 = TIM12->CCR2 = SHOT_FRIC_WHEEL_SPEED; 
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);		
		fric_wheel_run = 1;
	}
	else
	{
		TIM12->CCR1 = TIM12->CCR2 = 1000;
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);	
		fric_wheel_run = 0;
	}
	last_sw1 = rc.sw1;

}
void auto_aim_shoot(void)
{
	if (ReceData.visionDataStatus == 1)
	{
		//calculate pitch elevation
		YawBias = VisionDataX + AimDirectionBiasYaw;
		PitBias = VisionDataY + AimDirectionBiasPit;
		
		//you should modify these parameter
		//according your fitting data
		pnp_dis = VisionDataDis / 1000.0;
		AimDirectionBiasPit = (int)(21.7617
		                      - 36.7245 * pnp_dis
		                      + 5.8522 * pnp_dis * pnp_dis
		                      - 0.4506 * pnp_dis * pnp_dis * pnp_dis);
		
		PitchTarget =  pit_relative_angle + PitBias/RatioPitch;
		VAL_LIMIT(PitchTarget, -15, 17);
		LastPitchTarget = PitchTarget;
		

//		if (VisionDataX < 20 && VisionDataX > -20)
//		{
//			VisionDataX = VisionDataX/2;
//		}
		if (VisionDataX < 5 && VisionDataX > -5)
		{
			VisionDataX = 0;
		}
		
		YawTarget =  yaw_relative_angle + YawBias/RatioYaw;
		VAL_LIMIT(YawTarget, -60, 60);
		LastYawTarget = YawTarget;
		
		
		if((YawBias > -15) && (YawBias < 15))// && (VisionDataDis <= 5000))
		{
			auto_shoot_cmd = 1;
		}
		else
		{
			auto_shoot_cmd = 0;
		}
	}
	else if (ReceData.visionDataStatus == 2)
	{
		//?X  ?Y
		PitchTarget = VisionDataY * 57.3 / 1000.0;
		YawTarget   = VisionDataX * 57.3 / 1000.0;
		
		if((yaw_relative_angle - YawTarget > -2) 
			  && (yaw_relative_angle - YawTarget < 2))
		{
			auto_shoot_cmd = 1;
		}
		else
		{
			auto_shoot_cmd = 0;
		}
	}
	else
	{
		auto_shoot_cmd = 0;
		
		if (last_vision_status == 1)
		{
			no_vision_time = HAL_GetTick();
		}
		
		//if chassis follow gimbal when auto shooting lost vision
		//yaw return center immediately 
		if (auto_chassis_follow_gim == 1)
		{
			YawTarget = 0;
		}
		
		if (HAL_GetTick() - no_vision_time > 2000)
		{
			YawTarget = 0;
			PitchTarget = 0;
		}
	}
	
}
void getCaliData(void)
{
	if (gAppParam.GimbalCaliData.isAlreadyCalied == CALIED_FLAG)
	{
		yaw_center_offset = gAppParam.GimbalCaliData.GimbalYawOffset;
		pit_center_offset = gAppParam.GimbalCaliData.GimbalPitOffset;
	}
	else
	{
		//while (1);
	}
}
/**
  * @brief     gimbal control task
  * @attention the yaw angle feedback is single axis gyro, palstance feedback is mpu6500 gyro
  *            so maybe happen positive feedback, 
  *            if this case, you should modify their parameter sign.
  */
void gimbal_task(const void* argu)
{
	gimbal_pid_init();
	//mpu6500 & ist3810 magnet meter init ,for angular rate measure
	mpu_device_init();
	//read gimbal offset from gAppParam
	getCaliData();
//		can_cali_6025();
//		can_cali_6623();

	while (1)
	{
		//keep imu in a constant temperature
		imu_tmp = 21 + mpu_data.temp / 333.87f;
		pid_calc(&pid_imu_tmp, imu_tmp, imu_tmp_ref); //keep in 40 degree
		TIM3->CCR2 = pid_imu_tmp.pos_out;

		// get Z axis palstance, for gimbal speed loop
		mpu_get_data();
		gimbalCaliHook();

		if (gYaw.last_mode == GIMBAL_RELAX && gYaw.ctrl_mode != GIMBAL_RELAX)
			pit_angle_ref = 0; //back to center
		gYaw.last_mode = gYaw.ctrl_mode;

		switch (rc.sw2)
		{
			case (RC_UP):
				gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
				//gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ENCODER;
			break;
			case (RC_MI):
				gYaw.ctrl_mode = GIMBAL_RELAX;
			break;
			case (RC_DN):
				gYaw.ctrl_mode = GIMBAL_AUTO_SHOOT;
			break;
			default:
			break;
		}

		//shoot trigger moto task
		shoot_task();
		//transform absolute ecd range [0,8192] to relative range [-MOVE_RANGE, MOVE_RANGE],
		yaw_relative_pos = -get_relative_pos(moto_yaw.angle, yaw_center_offset);
		pit_relative_pos = get_relative_pos(moto_pit.angle, pit_center_offset);			
		
		pit_relative_angle = pit_relative_pos/22.75f;
		yaw_relative_angle = yaw_relative_pos/22.75f;	
	
		
		switch (gYaw.ctrl_mode)
		{
			case GIMBAL_AUTO_SHOOT:
			{
				if (gYaw.last_mode != GIMBAL_AUTO_SHOOT)
				{
					auto_shoot_pid_init();

				}
				pit_angle_fdb = pit_relative_angle;
				yaw_angle_fdb = yaw_relative_angle;
				
				if (chassis.mode == CHASSIS_AUTO)
				{	
					auto_aim_shoot();

					pit_angle_ref = PitchTarget;
					yaw_angle_ref = YawTarget;
					//chassis.is_snipe_mode = 1;

				}
				else
				{
					pit_angle_ref = 0;
					yaw_angle_ref = 0;
				}
			
			}break;
			
			case GIMBAL_CLOSE_LOOP_ZGYRO:
			{
				if (gYaw.last_mode != GIMBAL_CLOSE_LOOP_ZGYRO)
				{
					normal_pid_init();
			
				}
				pit_angle_fdb = pit_relative_angle;
				pit_angle_ref += rc.ch4 * 0.003f * GIMBAL_RC_MOVE_RATIO_PIT;
				
				if (chassis.mode == CHASSIS_FOLLOW_GIMBAL_ENCODER)
				{
					yaw_angle_fdb = yaw_zgyro_angle - gYaw.zgyro_offset;
					yaw_angle_ref += -rc.ch3 * 0.0015f * GIMBAL_RC_MOVE_RATIO_YAW;
				}
				else
				{
					yaw_angle_fdb = yaw_relative_angle;
					yaw_angle_ref = 0;					
				}
			}break;
			
			default:
			{
			}break;
		}

		
		
		VAL_LIMIT(pit_angle_ref, -10, 25);
		pit_speed_fdb = mpu_data.gx / 20.0f;
		pid_calc(&pid_pit, pit_angle_fdb, pit_angle_ref);
		if (gYaw.ctrl_mode == GIMBAL_AUTO_SHOOT)
			pit_speed_ref = (pid_pit.pos_out - pit_kp*mpu_data.gx) / 10.0f;
		else
			pit_speed_ref = pid_pit.pos_out / 10.0f;
		pid_calc(&pid_pit_speed, pit_speed_fdb, pit_speed_ref);
		
		yaw_speed_fdb = mpu_data.gz / 10.0f;
		pid_calc(&pid_yaw, yaw_angle_fdb, yaw_angle_ref);
		if (gYaw.ctrl_mode == GIMBAL_AUTO_SHOOT)
			yaw_speed_ref = (pid_yaw.pos_out - yaw_kp*mpu_data.gz) / 10.0f;
		else
			yaw_speed_ref = pid_yaw.pos_out / 10.0f;
		pid_calc(&pid_yaw_speed, yaw_speed_fdb, yaw_speed_ref);


		//pid_calc(&pid_trigger, moto_trigger.total_ecd / 100, trigger_pos_ref / 100);
		//pid_calc(&pid_trigger_speed, moto_trigger.speed_rpm, pid_trigger.pos_out);
		pid_calc(&pid_trigger_speed, moto_trigger.speed_rpm, trigger_spd_ref);

		//final output, for safe protect purpose
		if (rc.sw2 != RC_MI && gAppParam.GimbalCaliData.isAlreadyCalied == 0x55
				&& gYaw.ctrl_mode != GIMBAL_RELAX
				&& !gRxErr.err_list[DbusTOE].err_exist
				&& !gRxErr.err_list[GimbalYawTOE].err_exist
				&& !gRxErr.err_list[GimbalPitTOE].err_exist)
		{
			can_send_gimbal_iq(pid_yaw_speed.pos_out, -pid_pit_speed.pos_out, pid_trigger.pos_out);
		}
		else
		{
			pid_trigger.iout = 0;
			can_send_gimbal_iq(0, 0, 0); //relax state
		}
		
	  /* for debuge */
		pit_p_f = pit_angle_fdb * 1000;
		pit_p_r = pit_angle_ref * 1000;
		yaw_p_f = yaw_angle_fdb * 1000;
		yaw_p_r = yaw_angle_ref * 1000;

		osDelay(5);
	}
}
