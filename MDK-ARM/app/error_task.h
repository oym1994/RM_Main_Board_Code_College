/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       error_task.h
	* @brief      detect module offline or online  
	* @update   
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017  
	********************************(C) COPYRIGHT 2017 DJI************************
	*/

#include "stm32f4xx_hal.h"

#ifndef __ERROR_TASK_H__
#define __ERROR_TASK_H__

//TimeOutError = TOE id
typedef enum {
    NoTimeOutErr = 0,
    ChassisGyroTOE,
    ChassisMoto1TOE,
    ChassisMoto2TOE,
    ChassisMoto3TOE,
    ChassisMoto4TOE,
    CurrentTOE,
    DbusTOE,
    JudgeTOE,
    GimbalYawTOE,
    GimbalPitTOE,
    triggerMotoTOE,
    ErrorListLength,
} TOE_ID_e;


//#pragma pack(push)
#pragma pack(4)
typedef struct
{
    volatile uint32_t last_time;
    volatile uint32_t err_exist : 1; //1 = err_exist, 0 = everything ok
    volatile uint32_t enable : 1; 
    volatile uint32_t warn_pri : 6; //priority
    volatile uint32_t delta_time : 16; //time interval last
    volatile uint32_t set_timeout : 16;
    //void (*f_err_deal)(void); //error handle function pointer
} ErrorFlagTypedef;
#pragma pack()

typedef struct
{
    volatile ErrorFlagTypedef* err_now;
    volatile ErrorFlagTypedef  err_list[ErrorListLength];
} GlbRxErrTypeDef;

void global_err_detector_init(void);
void err_detector_hook(int err_id);
void detect_task(void const* argument);

extern GlbRxErrTypeDef gRxErr;

#endif
