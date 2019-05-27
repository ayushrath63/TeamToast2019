#ifndef __ECL_USER_H__
#define __ECL_USER_H__

extern uint32_t timer_semaphore;
#define ECL_TIMER_DISABLE_PROCESSING  __sync_fetch_and_add(&timer_semaphore, 1)
#define ECL_TIMER_ENABLE_PROCESSING   __sync_fetch_and_sub(&timer_semaphore, 1)
#define ECL_TIMER_PROCESSING_ENABLED  (__sync_fetch_and_add(&timer_semaphore, 0) == 0)

#endif //__ECL_USER_H__