
#ifndef _APA_LOG_H_
#define _APA_LOG_H_

#ifndef __cplusplus
#   error ERROR: This file requires C++ compilation(use a .cpp suffix)
#endif

#include <pthread.h>
#include <string>
#include <vector>
#include <iostream>
#include "string.h"
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "apa_module_id.h"
enum APA_LOG_LEVEL
{
    APA_LOG_LEVEL_ERROR = 0,
    APA_LOG_LEVEL_WARN,
    APA_LOG_LEVEL_INFO,
    APA_LOG_LEVEL_PERFORMANCE,
    APA_LOG_LEVEL_DEBUG,
    APA_LOG_LEVEL_MAX
};

enum APA_LOG_OUTPUT
{
    APA_LOG_OUTPUT_TERMINAL,
    APA_LOG_OUTPUT_FILE
};

class APA_Log
{
private:
    static APA_LOG_LEVEL SEARCH_LOG_ENABLE_LEVEL;
    static APA_LOG_OUTPUT output_type;
    static std::string output_path;
    static FILE* m_log_fp;
public:
    APA_Log() = default;
    ~APA_Log() = default;
    static void OutputClose();
    static void Output(APA_MODULE_ID id, APA_LOG_LEVEL level, const char *pkszInfoFormat, ...);
    static void Output_c(APA_MODULE_ID id, APA_LOG_LEVEL level, char* szLogInfo);

    static void SetEnableLevel(APA_LOG_LEVEL level) { SEARCH_LOG_ENABLE_LEVEL = level; }
    static void SetLogOutPath(APA_LOG_OUTPUT type, std::string path = std::string("/log/PatacLog/"), std::string file = std::string("apa.log")) { 
        output_type = type; 
        output_path = path;
        if (output_type == APA_LOG_OUTPUT_FILE){
        #if defined(__QNX__)
            if(1) {
        #else
            if(0 == access(output_path.c_str(), F_OK)) {
        #endif
                std::string log_file = output_path + "/" + file;
                m_log_fp = fopen(log_file.c_str(), "w");//zhguoi:"w"改为"a"
            }
            else {
                printf("APA_LOG output path [%s] not existed\n", output_path.c_str());
            }
        }
    }
};

#define __FILENAME__  (strrchr(__FILE__,'/') + 1)
#define SEARCHL_LOG_ENABLE

//#define SEARCHL_CORELOG_ENABLE
#define SEARCH_LOG_LEVEL_NORMAL  2
#define SEARCH_LOG_LEVEL_CRISIS  1
#define SEARCH_LOG_LEVEL_DANGER  0

// #ifndef SEARCH_LOG_ENABLE_LEVEL
// #define SEARCH_LOG_ENABLE_LEVEL  SEARCH_LOG_LEVEL_CRISIS
// #endif
#ifdef SEARCHL_LOG_ENABLE
    #define  APA_TIME   [](){time_t now = time(0);tm *tm_t = localtime(&now);     stringstream  time; \
            time<< "["<<tm_t->tm_year + 1900  <<"/"<< tm_t->tm_mon + 1 <<"/"<< tm_t->tm_mday<<"/" \
                << tm_t->tm_hour<<"/" << tm_t->tm_min<<"/"  << tm_t->tm_sec <<"]:"; \
                return time.str();}   
    #define APA_Trace_Log(MODULEID) (APA_Log::Output(MODULEID, APA_LOG_LEVEL_INFO, "File: %s, Function: %s, Line: %d",__FILENAME__,__FUNCTION__,__LINE__))
    #define APA_Info_Log(MODULEID, FORMAT,...) (APA_Log::Output(MODULEID, APA_LOG_LEVEL_INFO, FORMAT ,##__VA_ARGS__))
    #define APA_Error_Log(MODULEID, FORMAT, ...) (APA_Log::Output(MODULEID, APA_LOG_LEVEL_ERROR, FORMAT, ##__VA_ARGS__))
    #define APA_Debug_Log(MODULEID, FORMAT, ...) (APA_Log::Output(MODULEID, APA_LOG_LEVEL_DEBUG, FORMAT, ##__VA_ARGS__))
    #define APA_Performance_Log(MODULEID, FORMAT, ...) (APA_Log::Output(MODULEID, APA_LOG_LEVEL_PERFORMANCE, FORMAT, ##__VA_ARGS__))
    #ifdef SEARCHL_CORELOG_ENABLE
        #define APA_Core_Log(FORMAT,...) (APA_Log::Output(FORMAT ,##__VA_ARGS__))
    #else
        #define APA_Core_Log(FORMAT,...)
    #endif
    #define Divider(num, STR)  \
    do{\
    static uint32_t divide_num = 0;\
    divide_num += 1;       \
    divide_num %= num;     \
    if (divide_num == 1)   \
    {                      \
        STR;             \
    }\
    }while(0)

#else
    #define APA_Trace_Log() ((void)0)
    #define APA_Info_Log(FORMAT,...) ((void)0)
    #define APA_Error_Log(FORMAT,...) ((void)0)
    #define APA_Debug_Log(FORMAT,...) ((void)0)
    #define APA_Performance_Log(FORMAT,...) ((void)0)
    #define APA_Core_Log(FORMAT,...)
#endif
#endif
/* EOF */
