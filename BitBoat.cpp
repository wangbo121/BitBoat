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
      { SCHED_TASK(send_ap2gcs_realtime_data_boatlink),     100,    1000 },
      { SCHED_TASK(record_log),                             100,    1100 },
      { SCHED_TASK(get_timedata_now),                       100,    1100 },
      { SCHED_TASK(loop_slow),                              100,    1100 },
      { SCHED_TASK(end_of_task),                           1000,    1100 }
};

#define MAINTASK_TICK_TIME_MS 10//这个设置为10ms，对应每个循环100hz
int seconds=0;
int micro_seconds=MAINTASK_TICK_TIME_MS*(1e3);/*每个tick为20毫秒，也就是20000微秒*/
struct timeval maintask_tick;

struct T_GLOBAL_BOOL_BOATPILOT  global_bool_boatpilot;

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
    uint32_t timer = gettimeofday_us();//当前系统运行时间精确到微秒

    loop_fast();//在无人机中是姿态控制内环，在无人船中是导航控制环

    // tell the scheduler one tick has passed
    scheduler.tick();//告诉调度器scheduler一个tick已经过去了，目前1个tick指的是10毫秒

    /*
     * loop_us这个应该是一个tick循环所指定的时间，比如我这里目前定义的是10ms，
     * 但是如果所有任务的执行时间的总和还是小于10ms呢，如果没有select这个定时器，就会一直循环
     * 这样就会导致scheduler_tasks数组中指定的频率失去原本的意义，所以必须有select定时器或者
     * 如果在单片机中，则使用某一个定时器来触发这个loop这个函数
     */
    uint32_t loop_us = micro_seconds;
    uint32_t time_available = loop_us - ( gettimeofday_us() - timer );

    scheduler.run(time_available > loop_us ? 0u : time_available);
}

void Boat::loop_fast()
{
    /*
     * 这个loop_fast如果针对于飞机来说就是控制姿态的内环控制，无人机的导航控制环节在scheduler数组的执行中
     * 如果对于无人船来说，暂时作为导航和控制环
     */

    /*
     * 如果全部传感器都是硬件在环的，那么这里就不需要这个update_all_external_device_input
     * 如果传感器都硬件在环，那么就在数据有更新的时候，把数据填在这个结构中
     * 相当于在传感器和控制循环中间又添加了一层
     */
    update_all_external_device_input();

    /*1. decode_gcs2ap_radio*/
    decode_gcs2ap_radio();

    /*2. navigation*/
    navigation_loop(&auto_navigation,wp_data,&gps_data);

    /*3 control*/
    control_loop();
    /*
     * execute_ctrloutput是把control_loop的计算结果，实际映射到某个通道，比如rudder对应两个电机啥的
     * 而实际的输出放在
     */
    execute_ctrloutput(&ctrloutput);
}

void Boat::loop_slow()
{
    printf("hello loop_slow\n");

}

void Boat::end_of_task()
{

}

