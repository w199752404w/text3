
#ifndef __TYPE_H__
#define __TYPE_H__
#include <stdio.h>
#include "gd32f10x.h"


typedef enum{

    MI_FALSE  =   0,
    MI_TRUE   =   1,

}MI_BOOL;

typedef enum{
    
    MI_SUCCESS,
    MI_FAILURE, 
}MI_Status;

typedef uint8_t             MI_U8;
typedef char                MI_S8;
typedef uint16_t            MI_U16;
typedef short               MI_S16;
typedef uint32_t            MI_U32;
typedef uint64_t            MI_U64;

typedef int                 MI_S32;
typedef float               MI_FLOAT;
typedef double              MI_DOUBLE;
typedef void                MI_VOID;

typedef char                MI_CHAR;

#define MI_NULL             NULL

#define LOG_LEVEL_NONE 		0
#define LOG_LEVEL_ERROR 	1
#define LOG_LEVEL_NOTICE	2
#define LOG_LEVEL_WARNING 	3
#define LOG_LEVEL_INFO 		4
#define LOG_LEVEL_VERBOSE 	5

#define MAX_DEBUG_LEVEL LOG_LEVEL_INFO
#define APP_LINUX 1

#ifdef APP_LINUX

#define Max_Error(fmt,...){ \
    if(MAX_DEBUG_LEVEL >= LOG_LEVEL_ERROR) { \
	    printf("Max Error [func = %s] [line = %d] " fmt ,__FUNCTION__ , __LINE__,##__VA_ARGS__); \
    }\
}


#define Max_Info(fmt,...){ \
    if(MAX_DEBUG_LEVEL >= LOG_LEVEL_INFO) { \
	    printf("Max Info [func = %s]  " fmt ,__FUNCTION__,##__VA_ARGS__); \
    }\
}

#define Max_Warning(fmt,...){ \
    if(MAX_DEBUG_LEVEL >= LOG_LEVEL_WARNING) { \
	    printf("Max Warning [func = %s] [line = %d] " fmt ,__FUNCTION__ , __LINE__,##__VA_ARGS__); \
    }\
}

#define Max_Notice(fmt,...){ \
    if(MAX_DEBUG_LEVEL >= LOG_LEVEL_NOTICE) { \
	    printf("Max Notice [func = %s] [line = %d] " fmt ,__FUNCTION__ , __LINE__,##__VA_ARGS__); \
    }\
}

#else

#define Max_Error(fmt,...){ \
    if(MAX_DEBUG_LEVEL >= LOG_LEVEL_ERROR) { \
	    printf("Max Error [func = %s] [line = %d] " fmt ,__FUNCTION__ , __LINE__,##__VA_ARGS__); \
    }\
}


#define Max_Info(fmt,...){ \
    if(MAX_DEBUG_LEVEL >= LOG_LEVEL_INFO) { \
	    printf("Max Info [func = %s]  " fmt ,__FUNCTION__,##__VA_ARGS__); \
    }\
}

#define Max_Warning(fmt,...){ \
    if(MAX_DEBUG_LEVEL >= LOG_LEVEL_WARNING) { \
	    printf("Max Warning [func = %s] [line = %d] " fmt ,__FUNCTION__ , __LINE__,##__VA_ARGS__); \
    }\
}

#define Max_Notice(fmt,...){ \
    if(MAX_DEBUG_LEVEL >= LOG_LEVEL_NOTICE) { \
	    printf("Max Notice [func = %s] [line = %d] " fmt ,__FUNCTION__ , __LINE__,##__VA_ARGS__); \
    }\
}

#endif

#define CHECK_NULL_PTR(p)                       \
    do {                                        \
        if(MI_NULL == p)                        \
        {                                       \
            Max_Error("NULL pointer.\n");       \
            return MI_FAILURE;                  \
        }                                       \
       } while(0)

#define CHECK_STATUS(s)                         \
    do {                                        \
        if(MI_SUCCESS != s)                     \
        {                                       \
            printf("[Func]= %s, Status is Error,Please Check It.\n",__FUNCTION__);    \
            return MI_FAILURE;                  \
        }                                       \
       } while(0)
       
#endif
