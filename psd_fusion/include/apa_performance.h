
#ifndef CXX_apa_performance_H
#define CXX_apa_performance_H
#ifndef __cplusplus
#   error ERROR: This file requires C++ compilation (use a .cpp suffix)
#endif
#include <string>
/*---------------------------------------------------------------------------*/
/*    Reading of an include file                                             */

/*---------------------------------------------------------------------------*/
/*    Front declaration of a class                                           */
enum{
    SRCH_MAX_STRING_LENGTH = 1024
};
enum APA_PERFORMANCE_CHECK{
    APA_PERFORMANCE_CHECK_AUTO,
    APA_PERFORMANCE_CHECK_MANUAL
};
/*---------------------------------------------------------------------------*/
/*    Local definition declaration                                           */

class apa_performance
{
public:
    apa_performance(const char* strOutputString, APA_PERFORMANCE_CHECK ePerfromanceCheck = APA_PERFORMANCE_CHECK_AUTO);
    ~apa_performance();

    void
    Start();

    void
    Stop();

    void
    SetOutputString(std::string strOutputString);

private:

    long long GetSystemTime();
    void OutputLog();

    long long m_llStartTime;
    long long m_llEndTime;
    std::string m_strOutputString;
    APA_PERFORMANCE_CHECK m_ePerformanceCheck;
};

/*---------------------------------------------------------------------------*/
#endif /* CXX_apa_performance_H */
/* EOF */
