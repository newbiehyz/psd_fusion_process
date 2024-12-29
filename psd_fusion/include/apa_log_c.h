
#ifndef APA_LOG_C_H
#define APA_LOG_C_H

#include "string.h"
#include <sys/stat.h>
#include <unistd.h>
#include "apa_module_id.h"
#ifdef __cplusplus
extern "C" {
#endif 

    void output_apa_trace_log(APA_MODULE_ID id);
    void output_apa_info_log(APA_MODULE_ID id, const char *pkszInfoFormat, ...);
    void output_apa_error_log(APA_MODULE_ID id, const char *pkszInfoFormat, ...);
    void output_apa_debug_log(APA_MODULE_ID id, const char *pkszInfoFormat, ...);
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
#ifdef __cplusplus
}
#endif 

#define __FILENAME__  (strrchr(__FILE__,'/') + 1)
#define SEARCHL_LOG_ENABLE

#define SEARCH_LOG_LEVEL_NORMAL  2
#define SEARCH_LOG_LEVEL_CRISIS  1
#define SEARCH_LOG_LEVEL_DANGER  0

#endif
/* EOF */
