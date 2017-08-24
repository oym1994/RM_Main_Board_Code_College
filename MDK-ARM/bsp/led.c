#include "led.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

//void ledShowError(){
//	while(1){
//
//		#ifdef USE_FREE_RTOS
//			LED_R_ON;
//			osDelay(50);
//			LED_R_OFF;
//			osDelay(50);
//		#else
//			LED_R_ON;
//			HAL_Delay(50);
//			LED_R_OFF;
//			HAL_Delay(50);
//		#endif
//
//	}
//}
