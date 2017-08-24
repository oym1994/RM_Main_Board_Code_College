/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       calibrate.c
	* @brief      provides gimbal_offset/imu_data calibrate, 
	*             and save these calibration data in flash
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   richard.luo
  * @verbatim
	* 
	* @attention  
	* 
	********************************(C) COPYRIGHT 2017 DJI************************
	*/
#include "calibrate.h"
#include "bsp_can.h"
#include "bsp_flash.h"
AppParam_t gAppParam;

/**
  * @brief     save calibrate data gAppParam, write this structure in chip flash
  * @usage     called this function when calibrate data be changed
	*            in calibrate loop
  */
void AppParamSave2Flash(void)
{
    BSP_FLASH_Write((u8*)&gAppParam, sizeof(AppParam_t));
}
/**
  * @brief     read calibrate data gAppParam from chip flash
  * @usage     call this function after AppParamInit() in main() initialize part.
  * @attention use memcpy can replace the BSP_FLASH_Read()
  */
void AppParamReadFromFlash(void)
{
    memcpy((void*)&gAppParam, (void*)PARAM_SAVED_START_ADDRESS, sizeof(AppParam_t));
}

void AppParamInit(void)
{
    //so that we can see item name in debug mode.
    gAppParam.ImuCaliList[CALI_GYRO].name = "gyro";
    gAppParam.ImuCaliList[CALI_ACC].name  = "acc";
    gAppParam.ImuCaliList[CALI_MAG].name  = "mag";
}

/**
  * @brief     save gimbal pitch and yaw center point offset
	*            when parameter NeedCali = 1
	*            the offset is absolute encoder value
  * @param     none
  * @retval    none
  * @attention should be called in gimbal task loop 
  */
void gimbalCaliHook(void)
{
	if (gAppParam.GimbalCaliData.NeedCali == 1)
	{
		//absolute encoder angle[0,8191]
		gAppParam.GimbalCaliData.GimbalPitOffset = moto_pit.angle;
		gAppParam.GimbalCaliData.GimbalYawOffset = moto_yaw.angle;  
		gAppParam.GimbalCaliData.isAlreadyCalied = CALIED_FLAG;
		gAppParam.GimbalCaliData.NeedCali  = 0;
		AppParamSave2Flash();
	}
}

/**
  * @brief     save imu static offset 
  * @param[in] cali_id  : acceleration/palstance/compass
  * @param[in] raw_xyz[]: raw x axis data address
  * @retval    none
  * @attention called in mpu_get_data() function
  */
void imuCaliHook(CALI_ID_e cali_id, s16 raw_xyz[])
{
    static int sum[3];
    static int cnt = 0; //global / static var init = 0
    if (gAppParam.ImuCaliList[cali_id].NeedCali == 1)
    {
        gAppParam.ImuCaliList[cali_id].isAlreadyCalied = 0;
        sum[0] += raw_xyz[0];
        sum[1] += raw_xyz[1];
        sum[2] += raw_xyz[2];
        if (++cnt >= 100)
        {
            cnt                                      = 0;
            gAppParam.ImuCaliList[cali_id].offset[0] = sum[0] / 100.0f;
            gAppParam.ImuCaliList[cali_id].offset[1] = sum[1] / 100.0f;
            gAppParam.ImuCaliList[cali_id].offset[2] = sum[2] / 100.0f;
            sum[0] = sum[1] = sum[2]                       = 0;
            gAppParam.ImuCaliList[cali_id].NeedCali        = 0;
            gAppParam.ImuCaliList[cali_id].isAlreadyCalied = CALIED_FLAG;
            AppParamSave2Flash();
        }
    }
}
