#ifndef __WDG_H
#define	__WDG_H

#include "gd32f10x.h"


typedef enum
{
    EWdgType_fwdg,
    EWdgType_wwdg,
}EWdgType;

void gd32_wdgt_init(EWdgType type);
void gd32_wdgt_feed_dog(EWdgType type);

void Into_Standby_Mode(void);



#endif /* __WDG_H */

