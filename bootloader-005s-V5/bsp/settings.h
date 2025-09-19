#ifndef _SETTINGS_H__
#define _SETTINGS_H__

#include "type.h"

typedef struct settings
{
    /* data */
    MI_U8 update_flag;
    MI_U8 jump;
    MI_CHAR board_name[24];
    MI_CHAR soft_version[36];
    MI_CHAR settings_version[36];
    MI_CHAR app_version[36];
    MI_CHAR boot_version[36];
}system_info;


MI_BOOL system_info_get_update_flag(MI_U8 *flag);
MI_BOOL system_info_set_update_flag(MI_U8 flag);
MI_BOOL system_info_get_setting_version(MI_CHAR *version);
MI_BOOL system_info_get_app_version(MI_CHAR *version);
MI_BOOL system_info_get_broad_name(MI_CHAR *name);
#endif //_SETTINGS_H__
