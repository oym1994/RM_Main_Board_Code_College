/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       bsp_can.c
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

#include "bsp_can.h"
#include "can.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "error_task.h"

moto_measure_t moto_pit;
moto_measure_t moto_yaw;
moto_measure_t moto_trigger;
moto_measure_t moto_chassis[4] = { 0 }; //4 chassis moto
float yaw_zgyro_angle;

/**
  * @brief     get motor rpm and calculate motor round_count/total_encoder/total_angle
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after get_moto_offset() function
  */
void get_moto_measure(moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{
    ptr->speed_rpm     = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
   
    ptr->given_current = (int16_t)(hcan->pRxMsg->Data[4] << 8 | hcan->pRxMsg->Data[5]) / -5;
    ptr->hall          = hcan->pRxMsg->Data[6];
	
	  ptr->last_angle    = ptr->angle;
    ptr->angle         = (uint16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
	
    if (ptr->angle - ptr->last_angle > 4096)
		{
			ptr->round_cnt--;
			//ptr->ecd_raw_rate = ptr->angle - ptr->last_angle - 8192;
		}
    else if (ptr->angle - ptr->last_angle < -4096)
		{
      ptr->round_cnt++;
			//ptr->ecd_raw_rate = ptr->angle - ptr->last_angle + 8192;
		}
		else
		{
			//ptr->ecd_raw_rate = ptr->angle - ptr->last_angle;
		}
		//total encoder value
    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
	  //total angle/degree
		ptr->total_angle = ptr->total_ecd * 360 / 8192;
}

/**
  * @brief     get motor initialize offset value
  * @param     ptr: Pointer to a moto_measure_t structure
  * @retval    None
  * @attention this function should be called after system can init
  */
void get_moto_offset(moto_measure_t* ptr, CAN_HandleTypeDef* hcan)
{
    ptr->angle        = (uint16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
    ptr->offset_angle = ptr->angle;
}


/**
  * @brief   can filter initialization
  * @param   CAN_HandleTypeDef
  * @retval  None
  */
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
    //can1 &can2 use same filter config
    CAN_FilterConfTypeDef  CAN_FilterConfigStructure;
    static CanTxMsgTypeDef Tx1Message;
    static CanRxMsgTypeDef Rx1Message;
    static CanTxMsgTypeDef Tx2Message;
    static CanRxMsgTypeDef Rx2Message;

    CAN_FilterConfigStructure.FilterNumber         = 0;
    CAN_FilterConfigStructure.FilterMode           = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale          = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh         = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow          = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh     = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow      = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.BankNumber           = 14; //can1(0-13)and can2(14-27)each get half filter
    CAN_FilterConfigStructure.FilterActivation     = ENABLE;

    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
    {
        //err_deadloop(); 
    }

    //filter config for can2
    //can1(0-13),can2(14-27)
    CAN_FilterConfigStructure.FilterNumber = 14;
    if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
    {
        //err_deadloop();
    }

    if (_hcan == &hcan1)
    {
        _hcan->pTxMsg = &Tx1Message;
        _hcan->pRxMsg = &Rx1Message;
    }

    if (_hcan == &hcan2)
    {
        _hcan->pTxMsg = &Tx2Message;
        _hcan->pRxMsg = &Rx2Message;
    }
}

/**
  * @brief   callback this function when CAN interrupt happen.
  *          as: receives a correct CAN frame(gimbal/chassis esc...)
  * @param   hcan: Pointer to a CAN_HandleTypeDef structure that contains
  *          the configuration information for the specified CAN.  
  * @retval  None
  * @usage   add your can_device module receive api in this function, and communicate with 
  *          your module through can protocol
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
    //ignore can1 or can2.
	switch (_hcan->pRxMsg->StdId)
  {
    case CAN_3510Moto1_ID:
    case CAN_3510Moto2_ID:
    case CAN_3510Moto3_ID:
    case CAN_3510Moto4_ID:
    {
        static u8 i;
        i = _hcan->pRxMsg->StdId - CAN_3510Moto1_ID;

        moto_chassis[i].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[i], _hcan) : get_moto_measure(&moto_chassis[i], _hcan);
        err_detector_hook(ChassisMoto1TOE + i);
    }
    break;
    case CAN_YAW_FEEDBACK_ID:
    {
        get_moto_measure(&moto_yaw, _hcan);
        err_detector_hook(GimbalYawTOE);
    }
    break;
    case CAN_PIT_FEEDBACK_ID:
    {
        get_moto_measure(&moto_pit, _hcan);
        err_detector_hook(GimbalPitTOE);
    }
    break;
    case CAN_trigger_FEEDBACK_ID:
    {
        moto_trigger.msg_cnt++;

        moto_trigger.msg_cnt <= 10 ? get_moto_offset(&moto_trigger, _hcan) : get_moto_measure(&moto_trigger, _hcan);

        err_detector_hook(triggerMotoTOE);
    }
    break;

    case CAN_ZGYRO_FEEDBACK_MSG_ID:
    {
			yaw_zgyro_angle = -0.01f * ((s32)(_hcan->pRxMsg->Data[0] << 24) |       //0.01f * ((s32)(_hcan->pRxMsg->Data[0] << 24) | 
																			(_hcan->pRxMsg->Data[1] << 16) | 
																			(_hcan->pRxMsg->Data[2] << 8) | 
																			(_hcan->pRxMsg->Data[3]));
			err_detector_hook(ChassisGyroTOE);

    }
		break;
		
		case CAN_ZGYRO_CHASSIS_MSG_ID:
		{
			chassis.angle_from_gyro = 0.01f * ((s32)(_hcan->pRxMsg->Data[0] << 24) | 
																					(_hcan->pRxMsg->Data[1] << 16) | 
																					(_hcan->pRxMsg->Data[2] << 8) | 
																					(_hcan->pRxMsg->Data[3]));
			chassis.palstance_from_gyro = 0.01f * ((s32)(_hcan->pRxMsg->Data[4] << 24) | 
																							(_hcan->pRxMsg->Data[5] << 16) | 
																							(_hcan->pRxMsg->Data[6] << 8) | 
																							(_hcan->pRxMsg->Data[7]));
		}
    break;
		
		default:
		{
		}
		break;
	}

	//hcan1.Instance->IER|=0x00008F02;
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
}
