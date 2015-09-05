#ifndef _SYS_CONFIG_H
#define _SYS_CONFIG_H
//
#include "stm32f10x.h"

#define AUTO_MW
#define YAW_CORRECT
#define IMU_SW		//姿态解算使用软件解算，不再使用MPU6050的硬件解算单元DMP
#define HIGH_FREQ_CTRL
#define NEW_RC
#define lostRC_Landing
//#define STOP_MOTOR_FOREVER

#define UART_DEBUG	//开启改宏，则可以使用串口助手打印调试。否则使用Crazepony上位机

 
enum {SRC_PC,SRC_APP};
extern uint8_t btSrc;
//
//typedef struct SystemConfig_tt
//{
//	;
//}SystemConfig_t;

#ifdef UART_DEBUG
#define Q_printf(msg...)	printf(msg)
#else
#define	Q_printf(msg...)
#endif

/* --------------------------------------------- *
 *   adjustment / tuning / optimization          *
 *---------------------------------------------  */

/* XXX: MS5611 problems - pressure result is not stable.
 *
 * 1. The offset could be +-20cm (maybe worse) in a very short period that may lead 
 *	a. negetive 'Alititude'
 *	b. large 'GAP' during twice measurement
 * 2. In the long run, the value flucuate heavily. It may be caused by chip termp arising.
 *    For now, do not consider this case as we don't run it for a long time. Just keep it in mind.
 */
//#define MS5611_IGNORE_NEG	/* TBT: define it for resolving case 1.a */

/*
 * XXX: Pressure correction 1:
 *
 * step 1, to get the max delta between continuous twice fetching.
 * during idle state and manual movement, I got the value is '9'.
 *
 * step 2, set the threshhold value to one third of that value in step 1.
 * for each delta larger than threshhold, ignore this measure and keep the last value.
 *
 * DO NOT sure how it will impact when flying ... TO BE TESTED ...	
 */
//#define MS5611_PRESS_CORR_1	/* TBT: define it for resolving case 1.b */

#ifdef MS5611_PRESS_CORR_1
#define MAX_DELTA_PRESS	4
#endif

#endif

