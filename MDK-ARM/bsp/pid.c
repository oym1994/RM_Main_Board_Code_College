/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       pid.c
	* @brief      pid parameter initialization, position and delta pid calculate
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   Richard.luo        
  * @verbatim
	********************************(C) COPYRIGHT 2017 DJI************************
	*/
	
#include "pid.h"
#include "mytype.h"
#include <math.h>

#define ABS(x) ((x > 0) ? x : -x)
void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}

static void pid_param_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float    kp,
    float    ki,
    float    kd)
{

    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput     = maxout;
    pid->pid_mode      = mode;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}
/**
  * @brief     modify pid parameter when code running
  * @param[in] pid: control pid struct
  * @param[in] p/i/d: pid parameter
  * @retval    none
  */
static void pid_reset(pid_t* pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output 
  */
float pid_calc(pid_t* pid, float get, float set)
{
	pid->get[NOW] = get;
	pid->set[NOW] = set;
	pid->err[NOW] = set - get; 
	
	/* add max_err and deadband handle */
	if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)
			return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
			return 0;

	if (pid->pid_mode == POSITION_PID) //position PID
	{
			pid->pout = pid->p * pid->err[NOW];
			pid->iout += pid->i * pid->err[NOW];
			pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
			abs_limit(&(pid->iout), pid->IntegralLimit);
			pid->pos_out = pid->pout + pid->iout + pid->dout;
			abs_limit(&(pid->pos_out), pid->MaxOutput);
			pid->last_pos_out = pid->pos_out; //update last time
	}
	else if (pid->pid_mode == DELTA_PID) //delta PID
	{
			pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
			pid->iout = pid->i * pid->err[NOW];
			pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

			abs_limit(&(pid->iout), pid->IntegralLimit);
			pid->delta_u   = pid->pout + pid->iout + pid->dout;
			pid->delta_out = pid->last_delta_out + pid->delta_u;
			abs_limit(&(pid->delta_out), pid->MaxOutput);
			pid->last_delta_out = pid->delta_out; //update last time
	}

	pid->err[LLAST] = pid->err[LAST];
	pid->err[LAST]  = pid->err[NOW];
	pid->get[LLAST] = pid->get[LAST];
	pid->get[LAST]  = pid->get[NOW];
	pid->set[LLAST] = pid->set[LAST];
	pid->set[LAST]  = pid->set[NOW];
	
	return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}

/**
  * @brief     initialize pid parameter
  * @retval    none
  */
void PID_struct_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd)
{
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset  = pid_reset;

    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
}

pid_t pid_rol           = { 0 };
pid_t pid_pit           = { 0 };
pid_t pid_yaw           = { 0 };
pid_t pid_yaw_speed     = { 0 }; //yaw palstance loop
pid_t pid_pit_speed     = { 0 }; //pitch palstance loop
pid_t pid_spd[4]        = { 0 };
pid_t pid_chassis_angle = { 0 };
pid_t pid_trigger          = { 0 };
pid_t pid_trigger_speed      = { 0 };
pid_t pid_imu_tmp;

