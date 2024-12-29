/**
* Copyright @ 2020 - 2020 iAUTO(Shanghai) Co., Ltd.
* All Rights Reserved.
*
* Copyright @ 2020 - 2020 Pan Asia Technical Automotive Center Co., Ltd.
* All Rights Reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are NOT permitted except as agreed by
* iAUTO(Shanghai) Co., Ltd.
* or
* Pan Asia Technical Automotive Center Co., Ltd.
* *
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef APA_TIMER_H
#define APA_TIMER_H

#ifndef __cplusplus
#   error ERROR: This file requires C++ compilation(use a .cpp suffix)
#endif

#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <atomic>
#include "apa_module_id.h"

typedef enum {
	TIMER_SUCCESS = APA_ERRORCODE_BASE_APA_COMMON_LIB + 0,
	TIMER_CREATE_FAIL = APA_ERRORCODE_BASE_APA_COMMON_LIB + 1,
	TIMER_SET_FAIL = APA_ERRORCODE_BASE_APA_COMMON_LIB + 2,
}APA_Timer_Error_Code;

class apa_timer {
public:
	virtual ~apa_timer() {
		stopTimer();
	}
	apa_timer()
	{		 
		memset(&this->signalEvent, 0, sizeof(this->signalEvent));

		this->signalEvent.sigev_value.sival_ptr = (void*) this;
		this->signalEvent.sigev_notify = SIGEV_THREAD;
    		this->signalEvent.sigev_notify_function = timer_thread; 
		m_timer_deleted = false;

 		// Install the Timer
		if (0 != timer_create(CLOCK_REALTIME, &this->signalEvent, &this->timerID)) { 
			perror("Could not creat the timer");
			exit(1);
		}
	}
 
	void startTimer(APA_Timer_Error_Code& error_code,int cycle, bool loop_once = false) {
		if (m_timer_deleted) {
			return ;
		}

		// The itimerspec structure for the timer
		struct itimerspec timerSpecs;
		// Define the timer specification
		// One second till first occurrence
		timerSpecs.it_value.tv_sec = 0;
		timerSpecs.it_value.tv_nsec = cycle * 1000 * 1000;
		timerSpecs.it_interval.tv_sec = 0;
		if (loop_once) {
			timerSpecs.it_interval.tv_nsec = 0;
		}
		else {
			timerSpecs.it_interval.tv_nsec = cycle * 1000 * 1000;
		}

		// Set the timer and therefore it starts...
		if (timer_settime(timerID, 0, &timerSpecs, NULL) == -1) {
			error_code = TIMER_SET_FAIL;
			perror("Could not start timer:");
		}
	}
	void stopTimer() {

		if (!m_timer_deleted && this->timerID) {
			timer_delete(this->timerID);
			m_timer_deleted = true;
		}
	}

	static void timer_thread(union sigval v) {
		// get the pointer out of the siginfo structure and asign it to a new pointer variable
		apa_timer * ptrapa_timer = reinterpret_cast<apa_timer *> (v.sival_ptr);
		// call the member function
		ptrapa_timer->DoAction();
	}
	virtual void DoAction() = 0;

private:
	// Stored timer ID for alarm
	timer_t timerID;	 
	// The according signal event containing the this-pointer
	struct sigevent signalEvent;
	std::atomic_bool m_timer_deleted;
};
#endif
/* EOF */
