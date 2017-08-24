/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       chassis_task.c
	* @brief      provide basic chassis control, under remoter control have 
	*             chassis follow gimbal model and gimbal follow chassis model.
	*             auto control model have gimbal follow chassis model and 
	*             chassis gimbal separate modle.
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Jun-01-2017   Richard.luo      basic and auto control   
  * @verbatim
	*
	********************************(C) COPYRIGHT 2017 DJI************************
	*/
	
#include "chassis_task.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "can.h"
#include "cmsis_os.h"
#include "error_task.h"
#include "gimbal_task.h"
#include "judge_sys.h"
#include "kb.h"
#include "pid.h"
#include "mpu.h"

#include "stm32f4xx_hal.h"
#include "sys.h"
#include <math.h>
#define MyAbs(x) ((x > 0) ? (x) : (-x))
#define MAX_WHEEL_SPEED 2000
#define MAX_CHASSIS_VX_SPEED 900
#define MAX_CHASSIS_VY_SPEED 900
#define MAX_CHASSIS_VR_SPEED 500


#define r 0.076f
#define K 0.370f

#define SEND_SIZE 36  //sizeof(tSendTXoneData)
uint8_t auto_send[SEND_SIZE+1];

extern int last_sw1;

chassis_t chassis; // main chassis object

s16 buff_3510iq[4];

//kinetic solution parameter setting
double coef_total_angle = r*2*PI/(4.*19.*8191.);
double coef_speed_rpm = r*2*PI/(4.*19.*60.);
double last_d_x,last_d_y,last_a_w,d_x,d_y,a_w,diff_d_x,diff_d_y,diff_a_w,position_x,position_y,angle_w,angle_ww;
int16_t position_xs,position_ys,angle_ws,angle_ws2;
double v_x,v_y,w_v;
int16_t v_xs,v_ys,w_vs,v_xs2,v_ys2,w_vs2,w_vs3;


/**
  * @brief     reset chassis and gimbal single axis gyroscope
  * @attention gyro reset at least wait 2s  
  */
void reset_zgyro()
{
    while (ZGYRO_CAN.State == HAL_CAN_STATE_BUSY_TX);
    ZGYRO_CAN.pTxMsg->StdId   = CAN_ZGYRO_RST_ID;
    ZGYRO_CAN.pTxMsg->IDE     = CAN_ID_STD;
    ZGYRO_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
    ZGYRO_CAN.pTxMsg->DLC     = 0x08;
    ZGYRO_CAN.pTxMsg->Data[0] = 0;
    ZGYRO_CAN.pTxMsg->Data[1] = 1;
    ZGYRO_CAN.pTxMsg->Data[2] = 2;
    ZGYRO_CAN.pTxMsg->Data[3] = 3;
    ZGYRO_CAN.pTxMsg->Data[4] = 4;
    ZGYRO_CAN.pTxMsg->Data[5] = 5;
    ZGYRO_CAN.pTxMsg->Data[6] = 6;
    ZGYRO_CAN.pTxMsg->Data[7] = 7;
    HAL_CAN_Transmit(&ZGYRO_CAN, 1000);       
		
    while (CHASSIS_ZGYRO_CAN.State == HAL_CAN_STATE_BUSY_TX);
    CHASSIS_ZGYRO_CAN.pTxMsg->StdId   = CAN_ZGYRO_RST_ID;
    CHASSIS_ZGYRO_CAN.pTxMsg->IDE     = CAN_ID_STD;
    CHASSIS_ZGYRO_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
    CHASSIS_ZGYRO_CAN.pTxMsg->DLC     = 0x08;
    CHASSIS_ZGYRO_CAN.pTxMsg->Data[0] = 0;
    CHASSIS_ZGYRO_CAN.pTxMsg->Data[1] = 1;
    CHASSIS_ZGYRO_CAN.pTxMsg->Data[2] = 2;
    CHASSIS_ZGYRO_CAN.pTxMsg->Data[3] = 3;
    CHASSIS_ZGYRO_CAN.pTxMsg->Data[4] = 4;
    CHASSIS_ZGYRO_CAN.pTxMsg->Data[5] = 5;
    CHASSIS_ZGYRO_CAN.pTxMsg->Data[6] = 6;
    CHASSIS_ZGYRO_CAN.pTxMsg->Data[7] = 7;
	HAL_CAN_Transmit(&CHASSIS_ZGYRO_CAN, 1000); 
}

/**
	* @brief mecanum calculation function
  * @param input : vx vy vw(+ cw, - ccw)
  *        output: 4 wheel speed
	* @note  1=FR 2=FL 3=BL 4=BR
	* @map 	 2	%++++++%	1
	* 			 			++++
	* 						++++
	* 			 3	%++++++%	4    ↑=+Vy  →=+Vx
  */
void mecanum_calc(float vx, float vy, float vw, const int each_max_spd, s16 speed[])
{
	s16   buf[4];
	int   i;
	float max = 0, rate;

	VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
	VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);
	VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);

	buf[0] = (+vx - vy + vw);
	buf[1] = (+vx + vy + vw);
	buf[2] = (-vx + vy + vw);	
	buf[3] = (-vx - vy + vw);

	// find max item
	for (i = 0; i < 4; i++)
	{
			if (MyAbs(buf[i]) > max)
					max = MyAbs(buf[i]);
	}
	//equal proportion
	if (max > each_max_spd)
	{
			rate = each_max_spd / max;
			for (i = 0; i < 4; i++)
					buf[i] *= rate;
	}
	// output
	memcpy(speed, buf, sizeof(s16) * 4);
}
/**
  * @brief     send 4 calculated current to motor
  * @param     3510 motor ESC id
  * @retval    none
  */
void set_cm_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{

    hcan->pTxMsg->StdId   = 0x200;
    hcan->pTxMsg->IDE     = CAN_ID_STD;
    hcan->pTxMsg->RTR     = CAN_RTR_DATA;
    hcan->pTxMsg->DLC     = 0x08;
    hcan->pTxMsg->Data[0] = iq1 >> 8;
    hcan->pTxMsg->Data[1] = iq1;
    hcan->pTxMsg->Data[2] = iq2 >> 8;
    hcan->pTxMsg->Data[3] = iq2;
    hcan->pTxMsg->Data[4] = iq3 >> 8;
    hcan->pTxMsg->Data[5] = iq3;
    hcan->pTxMsg->Data[6] = iq4 >> 8;
    hcan->pTxMsg->Data[7] = iq4;
    HAL_CAN_Transmit(hcan, 1000);
}




void get_chassis_mode_set_ref(RC_Type* rc)
{

    chassis.last_mode = chassis.mode;

    switch (rc->sw1)
    {
    case RC_UP:
        chassis.mode = CHASSIS_AUTO;
        break;

    case RC_MI:
        chassis.mode = CHASSIS_OPEN_LOOP;
        break;

    case RC_DN:
				//chassis.mode = CHASSIS_CLOSE_LOOP_GYRO;
				chassis.mode = CHASSIS_FOLLOW_GIMBAL_ENCODER; 
        break;
    default:

        break;
    }

    switch (chassis.mode)
    {
			case CHASSIS_AUTO:
			{		
					//control coordinate is based on rc, where +y is forward, +x is right, +w is clockwise, 
				  //yet in the car coordinate, +y is left, +x is forward, +w is anti-clockwise.
					//auto_vx, auto_vy or auto_wv is measured by mm/s and deg/s
				  chassis.vy = 1 / (4*coef_speed_rpm)*(float)auto_vx/10000; 
					chassis.vx = -1/(4*coef_speed_rpm)*(float)auto_vy/10000;
					chassis.vw = -K /(4*coef_speed_rpm)*(float)auto_wv/180*PI/10;
			}break;
			
			case CHASSIS_OPEN_LOOP:
			{
					chassis.vy = rc->ch2 * CHASSIS_RC_MOVE_RATIO_Y + km.vy;
					chassis.vx = rc->ch1 * CHASSIS_RC_MOVE_RATIO_X + km.vx;
					chassis.vw = rc->ch3 / 2 + rc->mouse.x * 10; 
			}break;
			
			case CHASSIS_FOLLOW_GIMBAL_ENCODER: 
			{
					chassis.vy = rc->ch2 * CHASSIS_RC_MOVE_RATIO_Y + km.vy;
					chassis.vx = rc->ch1 * CHASSIS_RC_MOVE_RATIO_X + km.vx;
			}break;
			
			case CHASSIS_CLOSE_LOOP_GYRO:
			{
					chassis.vy = rc->ch2 * CHASSIS_RC_MOVE_RATIO_Y + km.vy;
					chassis.vx = rc->ch1 * CHASSIS_RC_MOVE_RATIO_X + km.vx;
					chassis.target_angle += -rc->ch3 * 0.001f;				
			}break;
			
			default :
			{
			}break;
    }
    // 四种状态
    // CHASSIS_AUTO：TX1,底盘跟随云台。CHASSIS_OPEN_LOOP：旋转轴跟随遥控器定点
    // CHASSIS_FOLLOW_GIMBAL_ENCODER: 遥控器，跟随云台。CHASSIS_CLOSE_LOOP_GYRO：底盘跟随遥控器旋转
}






void Measure_Position()
{
	 
	 //REMEMBER!!!
	 //because the board is bottom up so we need to change the original data ay (use but not send), wz an opposite sign.
	 
	 //1 feed back odom accumulation position and angle
	 last_d_x = d_x;
	 last_d_y = d_y;
	 last_a_w = a_w;
	 
	 d_x = coef_total_angle * (   moto_chassis[0].total_angle + moto_chassis[1].total_angle - moto_chassis[2].total_angle - moto_chassis[3].total_angle);
	 d_y = coef_total_angle * ( - moto_chassis[0].total_angle + moto_chassis[1].total_angle + moto_chassis[2].total_angle - moto_chassis[3].total_angle);
	 a_w = coef_total_angle / K *( moto_chassis[0].total_angle + moto_chassis[1].total_angle + moto_chassis[2].total_angle + moto_chassis[3].total_angle);
	 
	 diff_d_x = d_x - last_d_x;
	 diff_d_y = d_y - last_d_y;
	 diff_a_w = a_w - last_a_w;
	 
	 angle_ww = chassis.angle_from_gyro/57.3f; //deg from imu:zgyro	 
	 
	 position_x += diff_d_x*cos(angle_ww) - diff_d_y*sin(angle_ww);
	 position_y += diff_d_x*sin(angle_ww) + diff_d_y*cos(angle_ww);
	 angle_w += diff_a_w;
	 
	                            //coordinate transform
	 position_xs = (int16_t)( position_y*1000 );//mm from encoder
	 position_ys = (int16_t)(- position_x*1000);//mm from encoder
	 angle_ws = (int16_t)(- angle_w*180/PI);//deg from encoder
	 
	 angle_ws2 = (int16_t)(chassis.angle_from_gyro);//deg from zgyro    //
	 
	 //2 feed back velocity and angular
	 v_x = coef_speed_rpm * (   moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm);
	 v_y = coef_speed_rpm * ( - moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm + moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm);
	 w_v = coef_speed_rpm / K *(moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm + moto_chassis[2].speed_rpm + moto_chassis[3].speed_rpm);
	 
	 v_xs = (int16_t)( v_y*1000);  //mm/s from encoder                 //
	 v_ys = (int16_t)(- v_x*1000);//mm/s from encoder                  //
	 w_vs = (int16_t)(- w_v*180/PI);//deg/s from encoder
	 
	 v_xs2 = (int16_t)imu.vx;//mm/s from accx intergration            
	 v_ys2 = (int16_t)imu.vy;//mm/s from accy intergration
	 w_vs3 = (int16_t)( - mpu_data.gz / 16.384f); //deg from gyroz
	 w_vs2 = (int16_t)chassis.palstance_from_gyro;
	 
}




//int s_size, r_size;
void send_to_TXone(void)
{
	SendData.sof = 0xA5;
	SendData.angle = angle_ws2;
	SendData.v_w = w_vs2;
	SendData.v_x = v_xs;
	SendData.v_y = v_ys;
	SendData.flag = testGameInfo.gpsData.flag;
	SendData.x = testGameInfo.gpsData.x;
	SendData.y = testGameInfo.gpsData.y;
	SendData.z = testGameInfo.gpsData.z;
	SendData.compass = testGameInfo.gpsData.compass;
	SendData.yaw = yaw_relative_angle;
	SendData.pit = pit_relative_angle;
	SendData.end	= 0xFE;
	
//	s_size = sizeof(tSendTXoneData);
//	r_size = sizeof(tReceTXoneData);
	
	memcpy(auto_send, &SendData, sizeof(tSendTXoneData));
	HAL_UART_Transmit(&TX1_HUART, auto_send, sizeof(tSendTXoneData), 30);
}

/**
  * @brief     initialize chassis motor pid parameter
  * @usage     before chassis loop use this function
  */
void chassis_pid_param_init(void)
{
	for (int k = 0; k < 4; k++)
	{
		PID_struct_init(&pid_spd[k], POSITION_PID, 10000, 1000, 4, 0.05f, 5.0f);
	}
	PID_struct_init(&pid_chassis_angle, POSITION_PID, 600, 80, 1.0f, 0.0f, 0.0f);
	pid_chassis_angle.max_err  = 60 * 22.75f; // err angle > 60 cut the output
	pid_chassis_angle.deadband = 35;
}

int uart6_tx_count = 0;
void chassis_task(void const* argu)
{
	int i = 0;
	
	chassis_pid_param_init();

	HAL_Delay(1000);
    
	while (1)
	{
		pc_kb_hook();

		get_chassis_mode_set_ref(&rc);

		
		switch (chassis.mode)
		{
			case CHASSIS_CLOSE_LOOP_GYRO:
			{
				chassis.vw = -pid_calc(&pid_chassis_angle, chassis.angle_from_gyro, 
																	chassis.target_angle); 
			}break;
			
			case CHASSIS_FOLLOW_GIMBAL_ENCODER:
			{
				if (gYaw.ctrl_mode == GIMBAL_CLOSE_LOOP_ZGYRO)
				{
					chassis.vw = pid_calc(&pid_chassis_angle, yaw_relative_pos, 0); 
				}
				else
				{
					chassis.vw = 0;
				}
				
			}break;
			
			case CHASSIS_OPEN_LOOP:
			{
				if (gYaw.ctrl_mode == GIMBAL_AUTO_SHOOT)
					chassis.vw = 0;
			}break;
			
			case CHASSIS_AUTO:
			{
				if (ReceData.visionDataStatus == 1)
				{
					if (auto_chassis_follow_gim == 1)
					{
						chassis.vw = pid_calc(&pid_chassis_angle, yaw_relative_pos, 0); 
					}

				}
				
			}break;
			
			default:
			{
				chassis.vw = 0;
			}break;
		}

		if (chassis.is_snipe_mode)
				chassis.vw = 0;
		mecanum_calc(chassis.vx, chassis.vy, chassis.vw, MAX_WHEEL_SPEED,
								 chassis.wheel_speed);
		for (i = 0; i < 4; i++)
		{
				buff_3510iq[i] = pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm,
																	chassis.wheel_speed[i] * 10);
		}

		if (chassis.mode == CHASSIS_RELAX || gRxErr.err_list[DbusTOE].err_exist)
		{
				memset(buff_3510iq, 0, sizeof(buff_3510iq));
				pid_spd[0].iout = 0;
				pid_spd[1].iout = 0;
				pid_spd[2].iout = 0;
				pid_spd[3].iout = 0;
		}

		set_cm_current(&CHASSIS_CAN, buff_3510iq[0], buff_3510iq[1], buff_3510iq[2],
									 buff_3510iq[3]);

		
		uart6_tx_count++;
		
		if (uart6_tx_count >= 2)   //50Hz
		{
			
			Measure_Position();   // 编码器定位
			send_to_TXone();      // 发送数据给TX1
			uart6_tx_count = 0;
		}
		
		
		osDelay(10);
	}
}
