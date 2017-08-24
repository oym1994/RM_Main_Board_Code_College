/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       gimbal_task.h
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

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "mytype.h"
void gimbal_task(const void* argu);

typedef enum {

    GIMBAL_INIT = 0,
    GIMBAL_RELAX,
    //GIMBAL_CLOSE_LOOP_ENCODER,
	
		GIMBAL_AUTO_SHOOT,
    GIMBAL_CLOSE_LOOP_ZGYRO,

} eGimbalMode;

typedef struct
{
    eGimbalMode ctrl_mode; //yaw
    eGimbalMode last_mode; //

    float zgyro_target;
   	//float zgyro_rc;
   	//float zgyro_mouse;
    float zgyro_angle;
    float zgyro_offset;

} gimbal_yaw_t;

extern gimbal_yaw_t      gYaw;
extern s16   yaw_relative_pos;
extern float pit_relative_angle;
extern float yaw_relative_angle;
extern int   auto_chassis_follow_gim;
#endif
