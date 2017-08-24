/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       bsp_uart.c
	* @brief      uart receive data from DBus/bt/judge_system/manifold etc.
	* @update
  * @note       use DMA receive, but donot trigger DMA interrupt
	*             handle received data in usart idle interrupt handle function
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Jun-01-2017   Richard.luo      remove some useless module
  * @verbatim
	*		idle interrupt --> handle data --> clear it flag --> initialize DMA again
	*
	********************************(C) COPYRIGHT 2017 DJI************************
	*/

#include "bsp_uart.h"
#include "error_task.h"
#include "judge_sys.h"
#include "mytype.h"
#include "pid.h"
#include "sys.h"
#include "usart.h"


#define MAX_DMA_COUNT 100
#define DBUS_RX_MAX_BUFLEN 20
#define AUTOP_SIZE 15  //sizeof(tReceXToneTData)

//#define ARMAPI extern "C" // add this before hal callback

/* remote control data */
RC_Type rc;
u8      dbus_buff[DBUS_RX_MAX_BUFLEN];

/* tx1 vision data and autopilot data */
tSendTXoneData SendData;
tReceTXoneData ReceData;
uint8_t cv_buff[AUTOP_SIZE+1];

int16_t auto_vx, auto_vy, auto_wv;
int VisionDataX = 0;
int VisionDataY = 0;
int VisionDataDis = 0;
uint8_t VisionDataStatus = 0;
uint8_t last_vision_status;

/**
  * @brief   enable global uart it and do not use DMA transfer done it
  * @param   uart IRQHandler id, receive buff, buff size
  * @retval  set success or fail
  */
static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, u8* pData, u32 Size)
{
	uint32_t tmp1 = 0;

	tmp1 = huart->RxState;
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
				return HAL_ERROR;
		}

		/* Process Locked */
		__HAL_LOCK(huart);

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/* Enable the DMA Stream */
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,
									(uint32_t)pData, Size);

		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
	     in the UART CR3 register */
		huart->Instance->CR3 |= USART_CR3_DMAR;

		/* Process Unlocked */
		__HAL_UNLOCK(huart);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

/**
  * @brief   initialize uart device 
  * @usage   after MX_USARTx_UART_Init() use these function
  */
void judge_sys_init()
{
    __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
    UART_Receive_DMA_No_IT(&JUDGE_HUART, judge_buf, FRAME_BUFLEN);
}
void manifold_uart_init()
{
    __HAL_UART_CLEAR_IDLEFLAG(&TX1_HUART);
    __HAL_UART_ENABLE_IT(&TX1_HUART, UART_IT_IDLE);
    UART_Receive_DMA_No_IT(&TX1_HUART, cv_buff, AUTOP_SIZE);
}
void dbus_init()
{
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    UART_Receive_DMA_No_IT(&DBUS_HUART, dbus_buff, DBUS_RX_MAX_BUFLEN);
}

/**
  * @brief   clear idle it flag after uart receive a frame data
  * @param   uart IRQHandler id
  * @retval  none
  * @usage   call in MyUartFrameIRQHandler() function
  */
void uart_reset_idle_rx_callback(UART_HandleTypeDef* huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		// clear idle it flag
		uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx); 
		//according uart clear corresponding DMA flag

		__HAL_DMA_DISABLE(huart->hdmarx);
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAGS);
		__HAL_DMA_SET_COUNTER(huart->hdmarx, MAX_DMA_COUNT);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

void Uart_Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
{
    if (buff[0] == 0 && buff[1] == 0 && buff[2] == 0 && buff[3] == 0 && buff[4] == 0 && buff[5] == 0)
        return;

    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4 -= 1024;

    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);

    rc->mouse.l = buff[12]; //
    rc->mouse.r = buff[13];

    rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
}

/**
  * @brief   callback this function when uart interrupt 
  * @param   uart IRQHandler id
  * @retval  none
  * @usage   call in uart handler function USARTx_IRQHandler()
  */
void MyUartFrameIRQHandler(UART_HandleTypeDef* huart)
{
	if (huart == &DBUS_HUART)
	{
		Uart_Callback_RC_Handle(&rc, dbus_buff);
		err_detector_hook(DbusTOE);
	}
	else if (huart == &JUDGE_HUART)
	{
		judgementDataHandler();
	}

#ifdef TX1_HUART
	else if (huart == &TX1_HUART)
	{
		memcpy(&ReceData, cv_buff, sizeof(tReceTXoneData));
		
		if (ReceData.sof == 0xA5 && ReceData.end == 0xFE)
		{
			last_vision_status = VisionDataStatus;
			auto_vx = ReceData.auto_vx;
			auto_vy = ReceData.auto_vy;
			auto_wv = ReceData.auto_vw;
			
			VisionDataStatus = ReceData.visionDataStatus;
			VisionDataY      = ReceData.visionDataY;
			VisionDataX      = ReceData.visionDataX;
			VisionDataDis    = ReceData.visionDataDis;
		}

	}
#endif		
  uart_reset_idle_rx_callback(huart);
}

