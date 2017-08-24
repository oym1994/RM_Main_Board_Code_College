/********************************************************************************************************\
*                                     DJI System Global Macros Definations
*
*                                   (c) Copyright 2015; Dji, Inc.
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
\********************************************************************************************************/

#ifndef __SYS_H__
#define __SYS_H__

#include "can.h"
#include "mytype.h"
#include "usart.h"

#define FIRMWARE_VERSION #MAIN_VERSION##'.'
#define MAIN_VERSION 2
#define SUB_VERSION1 0
#define SUB_VERSION2 0


#define CHASSIS_CAN       hcan1
#define ZGYRO_CAN         hcan2
#define CHASSIS_ZGYRO_CAN hcan1
#define GIMBAL_CAN        hcan1

#define DBUS_HUART  huart1 //for dji remote controler reciever use
#define JUDGE_HUART huart3 //connected to judge system
#define TX1_HUART   huart6 //connected to TXone

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

/*================ CHASSIS MOVE SPEED RATIO ==================*/
#define CHASSIS_RC_MOVE_RATIO_X 1.0f //biger = move faster, 1.0f = 100%
#define CHASSIS_RC_MOVE_RATIO_Y 1.0f //biger = move faster, 1.0f = 100%
#define CHASSIS_PC_MOVE_RATIO_X 1.0f //biger = move faster, 1.0f = 100%
#define CHASSIS_PC_MOVE_RATIO_Y 1.0f //biger = move faster, 1.0f = 100%

//!!!#define CHASSIS_RC_ROTATE_RATIO			1.0f		//rotate rate is define by gimbal yaw move speed
//!!!#define CHASSIS_MOUSE_ROTATE_RATIO		1.0f		//rotate rate is define by gimbal yaw move speed
/*================ CHASSIS MOVE SPEED RATIO ==================*/

/*================ GIMBAL MOVE SPEED RATIO ==================*/
#define GIMBAL_RC_MOVE_RATIO_PIT 1.0f //biger = move faster, 1.0f = 100%
#define GIMBAL_RC_MOVE_RATIO_YAW 1.0f //biger = move faster, 1.0f = 100%

#define GIMBAL_PC_MOVE_RATIO_PIT 1.0f //biger = move faster, 1.0f = 100%
#define GIMBAL_PC_MOVE_RATIO_YAW 1.0f //biger = move faster, 1.0f = 100%
/*================ GIMBAL MOVE SPEED RATIO ==================*/

/*================ GIMBAL MOVE POSITION LIMIT ==================*/
#define YAW_MOVE_RANGE 3000 //from center(aka.0)， also = [center-RANGE, center+RANGE], total = 8192
#define PIT_MOVE_RANGE_MIN -14 //degree angle     from center(aka.0)， also = [center-RANGE, center+RANGE]
#define PIT_MOVE_RANGE_MAX 15
/*================ GIMBAL MOVE POSITION LIMIT ==================*/

/*================ GIMBAL SHOT PART PARAM ==================*/
#define trigger_MOTOR_SPEED 2000 //biger = faster
#define SHOT_FRIC_WHEEL_SPEED 1400 //biger = faster, max = 2000
/*================ GIMBAL SHOT PART PARAM ==================*/

/*================ Calibrate Variable Part ==================
global flash save var ==  [gAppParam]. conteins Gimbal + IMU +　Camera Offsets	
	@Cali how to cali 'GIMBAL'?	in debug mode. move gimbal pit+yaw to center.  
				set gAppParam.Gimbal.NeedCali = 1.
	@warning Remember : !!!Do NOT need to cali IMU
	
	@chinese 	UTF-8 encoding
					如何校准云台。使用jlink debug模式， 将云台扶正到中间 
					将gAppParam.Gimbal.NeedCali 设为1即可
================ Calibrate Variable Part ==================*/

/*================ Error Detect Part ==================*/
/**
	when error occurs Red LED will flash. otherwise Green LED lights.
	in debug mode, see variable [gRxErr] gRxErr.str = "xxxx" 
											eg: gyro lose => gRxErr.str"gyro"
	*/
/*================ Error Detect Part ==================*/
#endif

