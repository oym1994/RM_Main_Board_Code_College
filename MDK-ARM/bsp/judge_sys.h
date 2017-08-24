/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       judge_sys.h
	* @brief      read judgement system include blood/shoot information etc... 
	*             run judgementDataHandler() when usart idle interrupt
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   
  * @verbatim
	* FrameHeader(4-Byte) CmdID(2-Byte) Data(n-Byte) FrameTail(2-Byte,CRC16)
	* BaudRate:115200     WordLength:8Bits    Parity:None     StopBits:1
	* @attention  
	* if judgement system change the frame message, you should modify your code 
	* to match the judement frame.
	********************************(C) COPYRIGHT 2017 DJI************************
	*/

#ifndef __JUDGEMENT_H__
#define __JUDGEMENT_H__

#include "bsp_uart.h"
#include "mytype.h"
#include "stm32f4xx_hal.h"

#define FRAME_BUFLEN 100

/** 
  * @brief  命令码ID
  */
typedef enum {
    GameInfoId             = 0x0001, //比赛进程信息
    RealBloodChangedDataId = 0x0002, //实时血量变化数据
    RealShootDataId        = 0x0003, //实时射击数据
} comIdType;

/** 
  * @brief  FrameHeader structure definition
  */
typedef __packed struct
{
    uint8_t  sof;
    uint16_t dataLenth;
    uint8_t  seq;
    uint8_t  crc8;
} tFrameHeader;

/** 
  * @brief  GPS state structures definition
  */
typedef __packed struct
{
	  uint8_t  flag; //0:invalid 1:valid
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t compass;
} tGpsData;

/** 
  * @brief  Game information structures definition(0x0001)
  *         this package send frequency is 50Hz
  */
typedef __packed struct
{
    uint32_t remainTime; /*比赛剩余时间（从倒计时三分钟开始计算，单位 s）*/
    uint16_t remainLifeValue; /*机器人剩余血量*/
    float    realChassisOutV; /*实时底盘输出电压（单位 V）*/
    float    realChassisOutA; /*实时底盘输出电流（单位 A）*/
    tGpsData gpsData; /*GPS 状态， 见 tGpsData 结构体定义*/
    float    remainPower; //unit: J.//max = 60J
} tGameInfo;

/** 
  * @brief  实时血量变化信息(0x0002)
  */
typedef __packed struct
{
    uint8_t weakId : 4;
    /*0-3bits: 若变化类型为装甲伤害时：标识装甲 ID
                        0x00: 0 号装甲面 （前）
                        0x01： 1 号装甲面 （左）
                        0x02： 2 号装甲面 （后）
                        0x03： 3 号装甲面 （ 右）
                        0x04: 4 号装甲面 （上 1）
                        0x05: 5 号装甲面（ 上 2）*/

    uint8_t way : 4;
    /*4-7bits: 血量变化类型
                        0x0: 装甲伤害（受到攻击）
                        0x1：子弹超速扣血
                        0x2: 子弹超频扣血
                        0x3: 功率超限
                        0x4: 模块离线扣血
                        0x6: 普通犯规扣血
                        0xa: 获取加血神符
                        0xb: engineer auto recovery.*/

    uint16_t value; /*血量变化值*/
} tRealBloodChangedData;

/** 
  * @brief  实时射击信息(0x0003)
  */
typedef __packed struct
{
    float realBulletShootSpeed; //子弹实时射速（ m/s）
    float realBulletShootFreq; //子弹实时射频（ 发/s）
    float realGolfShootSpeed; //高尔夫实时射速(m/s 英雄机器人)
    float realGolfShootFreq; //高尔夫实时射频(发/s 英雄机器人)
} tRealShootData;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
} tStudentSelfDefineData;

typedef union {

    u8    U8[4];
    float F32;
} Send2PcDataConvertUnion;

extern uint8_t               judge_buf[];
extern tGameInfo             testGameInfo;
extern tRealBloodChangedData testRealBloodChangedData;
void                         judgementDataHandler(void);
//extern RC_UnionDef           tu_rc;

#endif
