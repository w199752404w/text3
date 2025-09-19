/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BSP_FUNCTION_H_
#define _BSP_FUNCTION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gd32f10x.h"
#include "main.h"

#ifdef USER_DEBUG
#define BSP_DEBUG_EN	0		/* 0:Non DEBUG status, 1:DEBUG status */
#else
#define BSP_DEBUG_EN	0
#endif
#define BSP_DEBUG(fmt,arg...)	do{if(BSP_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define BSP_RETURN_FALSE	do{BSP_DEBUG("Return failed");return false;}while(0)
//#define BSP_RETURN_TRUE do{BSP_DEBUG("Return success");return true;}while(0)
#define BSP_RETURN_TRUE do{return true;}while(0)

#define BSP_WDG_EN	0		//set to 0 when debugging, or set to 1

typedef enum {
	eBspWkInvalid = 0,	/* Ineffective wake-up method */
	eBspWkKey1,					/* Press button 1 to wake up */
	eBspWkKey2,					/* Press button 2 to wake up */
	eBspWkLoad,					/* Wake up under load */
	eBspWk232,					/* RS-232 wakes up */
	eBspWk485,					/* RS-485 wakes up */
	eBspWkCan,					/* Wake on CAN */
	eBspWkChg,					/* Charge to wake up */
	eBspWkExPwr					/* Wake up by external 12VDC power */
} BSP_WK_TYPE_E;

extern BSP_WK_TYPE_E g_eWkType;	/* Wake up mode */

extern void BSP_Init(void);
extern void BSP_WK_Detection(void);
//extern void BSP_POWER_DOWN_Detection(void);
extern void BSP_5V_1A_3A_Detection(void);
extern void BSP_Get_ADC1_Temp(void);
extern void BSP_Stop(void);

#ifdef __cplusplus
}
#endif 
#endif
