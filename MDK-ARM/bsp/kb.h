#ifndef __KB_H__
#define __KB_H__

#include "mytype.h"
#include "stm32f4xx_hal.h"

/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
//#define W 			0x0001		//bit 0
//#define S 			0x0002
//#define A 			0x0004
//#define D 			0x0008
//#define SHIFT 	0x0010
//#define CTRL 		0x0020
//#define Q 			0x0040
//#define E				0x0080
//#define R 			0x0100
//#define F 			0x0200
//#define G 			0x0400
//#define Z 			0x0800
//#define X 			0x1000
//#define C 			0x2000
//#define V 			0x4000		//bit 15
//#define B				0x8000
/******************************************************/

typedef enum {
    NormalMode = 0,
    ShiftQuicklyMode,
    CtrlSlowlyMode,
} KB_MoveMode;


//#define MAX_KB_SLOWLY_SPEED 150
//#define MAX_KB_NORMAL_SPEED 350
//#define MAX_KB_QUICKLY_SPEED 450

typedef struct
{
    float vx; //chassis move x speed
    float vy;
    int   acc;
    float qe_spin_angle; //Q/E spin not used yet
    u8    is_fire; //not used yet
    u8    fire_sta; //not used
} km_control_t;

extern km_control_t km;
void pc_kb_hook(void);
u8 key_fsm(u8* psta, u8 condition);
#endif
