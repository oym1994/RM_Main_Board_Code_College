/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       pid.h
	* @brief      pid parameter initialization, position and delta pid calculate
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   Richard.luo        
  * @verbatim
	********************************(C) COPYRIGHT 2017 DJI************************
	*/
	
#ifndef __pid_H__
#define __pid_H__

#include "stm32f4xx_hal.h"

enum
{
    LLAST = 0,
    LAST  = 1,
    NOW   = 2,
    POSITION_PID,
    DELTA_PID,
};
typedef struct __pid_t
{
    float p;
    float i;
    float d;

    float set[3]; //target NOW/LAST/LLAST
    float get[3]; //measure
    float err[3]; //error

    float pout; 
    float iout; 
    float dout; 

	  //position pid related
    float pos_out; //this time position output
    float last_pos_out; 
	
	  //delta pid related
    float delta_u; //this time delta value
    float delta_out; //this time delta output = last_delta_out + delta_u
    float last_delta_out;

	  ////deadband < err < max_err
    float    max_err;
    float    deadband; 
		
    uint32_t pid_mode;
    uint32_t MaxOutput;
    uint32_t IntegralLimit;

    void (*f_param_init)(struct __pid_t* pid, 
                         uint32_t        pid_mode,
                         uint32_t        maxOutput,
                         uint32_t        integralLimit,
                         float           p,
                         float           i,
                         float           d);
    void (*f_pid_reset)(struct __pid_t* pid, float p, float i, float d);
 
} pid_t;

void PID_struct_init(
	  pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd);

float pid_calc(pid_t* pid, float fdb, float ref);

extern pid_t pid_rol;
extern pid_t pid_pit;
extern pid_t pid_yaw;
extern pid_t pid_pit_speed;
extern pid_t pid_yaw_speed;
extern pid_t pid_spd[4];

extern pid_t pid_chassis_angle;
extern pid_t pid_trigger;
extern pid_t pid_trigger_speed;
extern pid_t pid_imu_tmp; //imu_temperature

#endif
