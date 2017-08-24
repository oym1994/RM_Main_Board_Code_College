/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       bsp_can.h
	* @brief      receive external can device message, motor_esc/gyroscope/module etc...
	*             get motor encoder initial offset, calculate motor speed 
	*             according to return encoder data
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   
  * @verbatim
	*   
	********************************(C) COPYRIGHT 2017 DJI************************
	*/
	
#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "can.h"
#include "mytype.h"

/* CAN send and receive ID */
typedef enum {

    CAN_TxGimbal_ID           = 0x1FF, //
    CAN_YAW_FEEDBACK_ID       = 0x205, //
    CAN_PIT_FEEDBACK_ID       = 0x206, //
    CAN_trigger_FEEDBACK_ID      = 0x207,
    CAN_ZGYRO_RST_ID          = 0x404,
    CAN_ZGYRO_FEEDBACK_MSG_ID = 0x401,
		CAN_ZGYRO_CHASSIS_MSG_ID  = 0x402,
	
    CAN_MotorLF_ID            = 0x041, 
    CAN_MotorRF_ID            = 0x042, 
    CAN_MotorLB_ID            = 0x043, 
    CAN_MotorRB_ID            = 0x044, 
    CAN_4Moto_Target_Speed_ID = 0x046, //
    CAN_GyroRecev_ID          = 0x011, //
    CAN_GyroReset_ID          = 0x012, //


    CAN_3510MotoAll_ID = 0x200,
    CAN_3510Moto1_ID   = 0x201,
    CAN_3510Moto2_ID   = 0x202,
    CAN_3510Moto3_ID   = 0x203,
    CAN_3510Moto4_ID   = 0x204,
    CAN_DriverPower_ID = 0x80,

    CAN_HeartBeat_ID = 0x156,

} eCAN_MSG_ID;

#define FILTER_BUF_LEN 5
/* can receive motor parameter structure */
typedef struct
{
    int16_t  speed_rpm;

    int16_t  given_current;
    uint8_t  hall;

    uint16_t offset_angle;
    int32_t  round_cnt;
    int32_t  total_ecd;
    int32_t  total_angle;
	  u32      msg_cnt;
	
	  uint16_t angle; //abs angle range:[0,8191]
    uint16_t last_angle; //abs angle range:[0,8191]
} moto_measure_t;


extern moto_measure_t moto_chassis[];
extern moto_measure_t moto_yaw, moto_pit, moto_trigger;
extern float          yaw_zgyro_angle;

void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);

#endif
