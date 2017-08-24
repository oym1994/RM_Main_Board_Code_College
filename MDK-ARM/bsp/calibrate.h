/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       calibrate.h
	* @brief      provides gimbal_offset/imu_data calibrate, 
	*             and save these calibration data in flash
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   
  * @verbatim
	* 
	* @attention  
	* 
	********************************(C) COPYRIGHT 2017 DJI************************
	*/

#ifndef __CALIBRATE_H__
#define __CALIBRATE_H__

#include "mytype.h"

#define CALIED_FLAG 0x55

typedef enum {
    CALI_GYRO,
    CALI_ACC,
    CALI_MAG,
    CALI_IMU_LIST_LEN,
    CALI_GIMBAL,
    //add more...
} CALI_ID_e;

typedef __packed struct
{
    int16_t GimbalYawOffset;
    int16_t GimbalPitOffset;
    uint8_t NeedCali;
    uint8_t isAlreadyCalied; //already calied
} GimbalCaliStruct_t;

typedef __packed struct
{
    int16_t offset[3]; //x,y,z
    uint8_t NeedCali; //1=need, 0=not
    uint8_t isAlreadyCalied; //0x55 = already calied, else not
    char*   name;
} ImuCaliStruct_t;

typedef __packed struct
{
    uint8_t            ParamSavedFlag; //header
    uint32_t           FirmwareVersion; //version
    GimbalCaliStruct_t GimbalCaliData; //gimbal pitch yaw encoder offset
    ImuCaliStruct_t    ImuCaliList[CALI_IMU_LIST_LEN]; // in fact =3
    GimbalCaliStruct_t CameraCali; //camera calibrate no use
} AppParam_t;

extern AppParam_t gAppParam;

void AppParamReadFromFlash(void);
void gimbalCaliHook(void);
void imuCaliHook(CALI_ID_e cali_id, s16 raw_xyz[]);
void AppParamInit(void);

#endif

