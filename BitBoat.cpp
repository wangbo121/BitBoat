/*
 * BitBoat.cpp
 *
 *  Created on: Nov 4, 2017
 *      Author: wangbo
 */

#include "Boat.h"

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
  1 = 100hz
  2 = 50hz
  4 = 25hz
  100 = 1hz
  中间的数字是执行频率，也就是经过多少个tick(目前我写的是10ms一个tick)执行一次这个任务
 */
#define SCHED_TASK(func) (void (*)())&Boat::func
const AP_Scheduler::Task Boat::scheduler_tasks[] = {
//    { update_GPS,            2,     900 },
//    { update_navigation,     10,    500 },
//    { medium_loop,           2,     700 },
//    { update_altitude,      10,    1000 },
//    { fifty_hz_loop,         2,     950 },
//    { run_nav_updates,      10,     800 },
//    { slow_loop,            10,     500 },
//    { gcs_check_input,       2,     700 },
//    { gcs_send_heartbeat,  100,     700 },
//    { gcs_data_stream_send,  2,    1500 },
//    { gcs_send_deferred,     2,    1200 },
//    { compass_accumulate,    2,     700 },
//    { barometer_accumulate,  2,     900 },
//    { super_slow_loop,     100,    1100 },
//    { perf_update,        1000,     500 }
//      { (void (*)())&Boat::loop_slow,     100,    1100 },
      { SCHED_TASK(loop_slow),     100,    1100 },
      { SCHED_TASK(loop_super_slow),     1000,    1100 }
//      { test,     1000,    1100 }
};

#define MAINTASK_TICK_TIME_MS 10//这个设置为10ms，对应每个循环100hz
int seconds=0;
int micro_seconds=MAINTASK_TICK_TIME_MS*(1e3);/*每个tick为20毫秒，也就是20000微秒*/
struct timeval maintask_tick;

int main(int argc,char * const argv[])
{
    DEBUG_PRINTF("Welcome to BitPilot\n");

    // initialise the main loop scheduler
    boat.scheduler.init(&boat.scheduler_tasks[0], sizeof(boat.scheduler_tasks)/sizeof(boat.scheduler_tasks[0]));
    printf(" sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]) = %d\n",sizeof(boat.scheduler_tasks)/sizeof(boat.scheduler_tasks[0]));

    //初始化工作
    boat.setup();

    while (1)
    {
        maintask_tick.tv_sec = seconds;
        maintask_tick.tv_usec = micro_seconds;
        select(0, NULL, NULL, NULL, &maintask_tick);

        /*
         * 如果这个while(1)的循环周期是10ms那么
         * 这个loop循环中所有的函数都执行一边（或者说运行最多函数时），所需要的时间应该是小于10ms的
         * 如果所有任务都执行一遍，所需要的时间还是小于10ms呢，会停在那里等待吗？
         */
        boat.loop();
    }

    return 0;
}

void Boat::loop( void )
{
    loop_cnt++;
    if(loop_cnt % 100 == 0)
    {
        printf("loop_cnt = %d \n",loop_cnt);//1秒钟显示一次循环计数
    }

    uint32_t timer = gettimeofday_us();

    // Execute the fast loop
    loop_fast();

    // tell the scheduler one tick has passed
    scheduler.tick();

    //fast_loop_finish_timer = gettimeofday_us();

    /*
     * loop_us这个应该是一个tick循环所指定的时间，比如我这里目前定义的是10ms，
     * 但是如果所有任务的执行时间的总和还是小于10ms呢，如果没有select这个定时器，就会一直循环
     * 这样就会导致scheduler_tasks数组中指定的频率失去原本的意义，所以必须有select定时器或者
     * 如果在单片机中，则使用某一个定时器来触发这个loop这个函数
     */
    //uint16_t dt = timer - fast_loop_finish_timer;
    //uint16_t dt = fast_loop_finish_timer - timer;
    //uint32_t loop_us = 10000;//10ms
    uint32_t loop_us = 9000;//10ms
    loop_us = micro_seconds;
    //uint32_t time_available= timer + loop_us - gettimeofday_us();
    uint32_t time_available = loop_us - ( gettimeofday_us() - timer );

    scheduler.run(time_available > loop_us ? 0u : time_available);
}

void Boat::loop_fast()
{
    //printf("hello loop_fast\n");
}

void Boat::loop_slow()
{
    printf("hello loop_slow\n");

}

void Boat::loop_super_slow()
{
    printf("hello loop_super_slow\n");
}
