/**
 * @file    version.c
 * @brief   SDK 版本信息
 */
#include "miraculous_sdk.h"

const char* miraculous_sdk_version(void)
{
    return MIRACULOUS_SDK_VERSION;
}

const char* miraculous_sdk_build_time(void)
{
    return __DATE__ " " __TIME__;
}
