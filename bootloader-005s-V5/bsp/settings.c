#include <string.h>
#include "settings.h"
#include "flash.h"

MI_BOOL system_info_get_setting_version(MI_CHAR *version)
{
    system_info info;

    memset(&info,0,sizeof(info));
    gd32_flash_read(SETTINGS_START_ADDRESS,(uint8_t *)&info,sizeof(info));
    memcpy(version,info.settings_version,sizeof(info.settings_version));

    return MI_TRUE;
}

MI_BOOL system_info_get_app_version(MI_CHAR *version)
{
    system_info info;

    memset(&info,0,sizeof(info));
    gd32_flash_read(SETTINGS_START_ADDRESS,(uint8_t *)&info,sizeof(info));
    memcpy(version,info.app_version,sizeof(info.app_version));

    return MI_TRUE;
}

MI_BOOL system_info_get_broad_name(MI_CHAR *name)
{
    system_info info;

    memset(&info,0,sizeof(info));
    gd32_flash_read(SETTINGS_START_ADDRESS,(uint8_t *)&info,sizeof(info));
    memcpy(name,info.board_name,sizeof(info.board_name));

    return MI_TRUE;
}

MI_BOOL system_info_get_update_flag(MI_U8 *flag)
{
    system_info info;

    memset(&info,0,sizeof(info));
    gd32_flash_read(SETTINGS_START_ADDRESS,(uint8_t *)&info,sizeof(info));
    *flag = info.update_flag;

    return MI_TRUE;
}

/**
 * 设置升级标志位的函数
*/
MI_BOOL system_info_set_update_flag(MI_U8 flag)
{
    system_info info;

    memset(&info,0,sizeof(info));
    gd32_flash_read(SETTINGS_START_ADDRESS,(uint8_t *)&info,sizeof(info));
    info.update_flag = flag;

    //先清除system info 区域
    gd32_flash_erase(SETTINGS_START_ADDRESS,SETTINGS_END_ADDRESS);
    gd32_flash_write(SETTINGS_START_ADDRESS,(uint8_t *)&info,sizeof(info));

    return MI_TRUE;
}
