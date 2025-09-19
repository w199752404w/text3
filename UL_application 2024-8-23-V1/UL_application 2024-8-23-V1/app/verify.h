/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _VERIFY_H_
#define _VERIFY_H_

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include "gd32f10x.h"
#include "main.h"

#ifdef USER_DEBUG
#define VERIFY_DEBUG_EN	0		/* 0:Non DEBUG status, 1:DEBUG status */
#else
#define VERIFY_DEBUG_EN	1
#endif
#define VERIFY_DEBUG(fmt,arg...)	do{if(VERIFY_DEBUG_EN){printf(fmt,##arg);printf("[%s][%d]\r\n",__func__,__LINE__);}}while(0)
#define VERIFY_RETURN_FALSE	do{VERIFY_DEBUG("Return failed");return false;}while(0)
//#define VERIFY_RETURN_TRUE do{VERIFY_DEBUG("Return success");return true;}while(0)
#define VERIFY_RETURN_TRUE do{return true;}while(0)

extern void verify(void);

#ifdef __cplusplus
}
#endif 
#endif
