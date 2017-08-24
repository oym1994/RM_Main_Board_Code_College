/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       error_task.c
	* @brief      detect module offline or online  
	* @update   
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017  
	********************************(C) COPYRIGHT 2017 DJI************************
	*/
	
#include "error_task.h"
#include "cmsis_os.h"
#include "led.h"
#include "mytype.h"
#include "stm32f4xx_hal.h"

GlbRxErrTypeDef gRxErr;
u32 systime;

/**
  * @brief     initialize detector error_list
  * @usage     used before detect loop in detect_task() function
	*            the offline judgment standard is set_timeout
	*            you can set different offline priority(warn_pri) to deal with module conflict
  */
void global_err_detector_init(void)
{
    gRxErr.err_now = NULL;

    gRxErr.err_list[DbusTOE].err_exist = 0;
    gRxErr.err_list[DbusTOE].warn_pri    = 10; //max priority
    gRxErr.err_list[DbusTOE].set_timeout = 100; //ms
    gRxErr.err_list[DbusTOE].delta_time  = 0;
    gRxErr.err_list[DbusTOE].last_time   = 0x00;
    gRxErr.err_list[DbusTOE].enable      = 1;

    for (int i = 0; i < 4; i++)
    {
			gRxErr.err_list[ChassisMoto1TOE + i].err_exist = 0;
			gRxErr.err_list[ChassisMoto1TOE + i].warn_pri    = 6 + i; //6,7,8,9
			gRxErr.err_list[ChassisMoto1TOE + i].set_timeout = 200;
			gRxErr.err_list[ChassisMoto1TOE + i].delta_time  = 0;
			gRxErr.err_list[ChassisMoto1TOE + i].last_time   = 0x00;
			gRxErr.err_list[ChassisMoto1TOE + i].enable      = 1;
    }

    gRxErr.err_list[ChassisGyroTOE].err_exist = 0;
    gRxErr.err_list[ChassisGyroTOE].warn_pri    = 5;
    gRxErr.err_list[ChassisGyroTOE].set_timeout = 10;
    gRxErr.err_list[ChassisGyroTOE].delta_time  = 0;
    gRxErr.err_list[ChassisGyroTOE].last_time   = 0x00;
    gRxErr.err_list[ChassisGyroTOE].enable      = 1;

//    gRxErr.err_list[CurrentTOE].err_exist = 0;
//    gRxErr.err_list[CurrentTOE].warn_pri    = 4;
//    gRxErr.err_list[CurrentTOE].set_timeout = 200;
//    gRxErr.err_list[CurrentTOE].delta_time  = 0;
//    gRxErr.err_list[CurrentTOE].last_time   = 0x00;
//    gRxErr.err_list[CurrentTOE].enable = 1;

    gRxErr.err_list[GimbalPitTOE].err_exist   = 0;
    gRxErr.err_list[GimbalPitTOE].warn_pri    = 8;
    gRxErr.err_list[GimbalPitTOE].set_timeout = 200;
    gRxErr.err_list[GimbalPitTOE].delta_time  = 0;
    gRxErr.err_list[GimbalPitTOE].last_time   = 0x00;
    gRxErr.err_list[GimbalPitTOE].enable      = 1;

    gRxErr.err_list[GimbalYawTOE].err_exist   = 0;
    gRxErr.err_list[GimbalYawTOE].warn_pri    = 8;
    gRxErr.err_list[GimbalYawTOE].set_timeout = 200;
    gRxErr.err_list[GimbalYawTOE].delta_time  = 0;
    gRxErr.err_list[GimbalYawTOE].last_time   = 0x00;
    gRxErr.err_list[GimbalYawTOE].enable      = 1;

    gRxErr.err_list[triggerMotoTOE].err_exist   = 0;
    gRxErr.err_list[triggerMotoTOE].warn_pri    = 2;
    gRxErr.err_list[triggerMotoTOE].set_timeout = 200;
    gRxErr.err_list[triggerMotoTOE].delta_time  = 0;
    gRxErr.err_list[triggerMotoTOE].last_time   = 0x00;
    gRxErr.err_list[triggerMotoTOE].enable      = 1;
}

/**
  * @brief     record the detected module return time to judge offline
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */
void err_detector_hook(int err_id)
{
    if (gRxErr.err_list[err_id].enable)
        gRxErr.err_list[err_id].last_time = HAL_GetTick();
}

/**
  * @brief     according to the interval time
  * @param     err_id: module id
  * @retval    None
  * @usage     used in CAN/usart.. rx interrupt callback
  */
void detect_task(void const* argument)
{
	global_err_detector_init();
	osDelay(100);

	while(1)
	{
		systime = HAL_GetTick();
		int max_priority = 0;
		int err_cnt      = 0;
		for (int id = 0; id < ErrorListLength; id++)
		{
			gRxErr.err_list[id].delta_time = HAL_GetTick() - gRxErr.err_list[id].last_time;
			if (gRxErr.err_list[id].enable && (gRxErr.err_list[id].delta_time > gRxErr.err_list[id].set_timeout))
			{
				gRxErr.err_list[id].err_exist = 1; //this module is offline
				err_cnt++;
				if (gRxErr.err_list[id].warn_pri > max_priority)
				{
					max_priority   = gRxErr.err_list[id].warn_pri;
					gRxErr.err_now = &(gRxErr.err_list[id]);
				}
			}
			else
			{
				gRxErr.err_list[id].err_exist = 0;
			}
		}

		if (!err_cnt) //all scan no error, should clear err pointer!!!
			gRxErr.err_now = NULL;

		if (gRxErr.err_now != NULL)
		{
			//run your offline handle function
			LED_G_OFF;
		}
		else
		{
			LED_G_ON;
		}
		osDelay(50);
	}
}

//void set_led(int led)
//{
//    LED_PORT->ODR &= ~(7 << 1); 
//    LED_PORT->ODR |= ~(led << 1);
//}

//void err_deadloop()
//{
//    set_led(LED_R | LED_B);
//    while (1)
//    {
//        set_led(LED_R | LED_B);
//        HAL_Delay(300);
//        set_led(LED_R | LED_B);
//        HAL_Delay(1000);
//    }
//}
