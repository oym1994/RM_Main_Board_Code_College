/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       mpu.h
	* @brief      mpu6050/mpu6500/mpu9250 module driver.
  *             Configuration MPU6500 or MPU 9250 and Read the Accelerator
	*             and Gyrometer data using SPI interface
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Nov-19-2015   langgo.chen      
  * @verbatim
	*   
	********************************(C) COPYRIGHT 2017 DJI************************
	*/


#ifndef __MPU_H__
#define __MPU_H__

#include "mytype.h"

typedef struct
{
    s16 ax;
    s16 ay;
    s16 az;

    s16 mx;
    s16 my;
    s16 mz;

    s16 temp;

    s16 gx;
    s16 gy;
    s16 gz;
		
		s16 ax_offset;
		s16 ay_offset;
		s16 az_offset;
	
		s16 gx_offset;
		s16 gy_offset;
		s16 gz_offset;
		
} MPU_OriginData;

typedef struct
{
    s16 ax;
    s16 ay;
    s16 az;

    s16 mx;
    s16 my;
    s16 mz;

    float temp;
	
    float wx; //omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s
    float wy;
    float wz;

		float vx;
		float vy;
		float vz;
	
    float rol;
    float pit;
    float yaw;
} imu_t;

u8          mpu_device_init(void);
void        mpu_get_data(void);
void 			  mpu_offset_cal(void);
extern MPU_OriginData mpu_data;
extern imu_t          imu;

#endif
