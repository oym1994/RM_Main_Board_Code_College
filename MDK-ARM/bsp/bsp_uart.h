/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       bsp_uart.h
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

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "mytype.h"
#include "usart.h"

#pragma pack()
typedef struct {
	//ch value: -364 ~ 364
	int16_t ch1;	
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	
	//switch val: 1 3 2
	uint8_t sw1;	
	uint8_t sw2;
	
	struct {
		int16_t x;
		int16_t y;
		int16_t z;//no use
	
		//press:1 release:0
		uint8_t l;
		uint8_t r;
	}mouse;
	
	union {
		uint16_t key_code;
/**********************************************************************************
 * keyboard :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 *            V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 ************************************************************************************/
		struct {
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		}bit;
	}kb;
	
}RC_Type;



enum{
	RC_UP = 1,
	RC_MI = 3,
	RC_DN = 2,
};


typedef __packed struct
{
	uint8_t sof; //0xA5
	int16_t auto_vx;
	int16_t auto_vy;
	int16_t auto_vw;
	uint8_t	visionDataStatus;
	int16_t visionDataX;
	int16_t visionDataY;
	int16_t visionDataDis;
	uint8_t end; //0xFE
} tReceTXoneData;

typedef __packed struct
{
	uint8_t sof;      //0xA5
	int16_t angle;    //chassis angle
	int16_t v_w;      //chassis angluar rate  deg/s
	int16_t v_x;      //x velocity  mm/s
	int16_t v_y;      //y velocity  mm/s

  uint8_t  flag;    //uwb 1:useful 0:useless
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint32_t compass;
	uint8_t dataStatus;  //
	float yaw;
	float pit;
	uint8_t end;      //0xFE
} tSendTXoneData;


extern RC_Type rc;

extern tSendTXoneData SendData;
extern tReceTXoneData ReceData;

extern int16_t auto_vx, auto_vy, auto_wv;

extern int VisionDataX;
extern int VisionDataY;
extern int VisionDataDis;
extern uint8_t VisionDataStatus;
extern uint8_t last_vision_status;

void dbus_init(void);
void manifold_uart_init(void);
void judge_sys_init(void);

void MyUartFrameIRQHandler(UART_HandleTypeDef *huart);

#endif
