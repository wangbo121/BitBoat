/*
 * BitBoat.cpp
 *
 *  Created on: Nov 4, 2017
 *      Author: wangbo
 */

#include "Boat.h"

/*
 * 仿真测试使用
 */
//Watercraft sim_water_craft("32.68436,117.05525,10,0","+");//+型机架，起始高度为10，yaw是0
//Watercraft sim_water_craft("39.95635,116.31574,10,0","+");//+型机架，起始高度为10，yaw是0

/*
 * 这是任务调度表，除了fast_loop中的任务，其他任务都在这里执行
 * 中间的数字是执行频率，也就是经过多少个tick执行一次这个任务(目前我写的是10ms一个tick)
 * 最右边的数字是最大允许的时间，单位是微妙
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
  1 = 100hz
  2 = 50hz
  4 = 25hz
  10 = 10hz
  100 = 1hz
  每个函数的执行时间必须小于最大允许的时间，单位是微秒，否则就会出现Scheduler overrun task
  因此需要特别注意串口读取 udp读取等阻塞式的读取方式，阻塞等待的时间需要小于这个任务调度允许的最大时间
  gps_Y901我给的最大允许是3000微秒，而gps_Y901是在10hz循环中的，因此该10hz循环中最多能有3个gps_Y901这种
  读取函数，否则就会把10ms全部占用了
 */
#define SCHED_TASK(func) (void (*)())&Boat::func

const BIT_Scheduler::Task Boat::scheduler_tasks[] =
{
    //真正读取传感器函数
    //{ SCHED_TASK(read_device_gps_JY901),                                                  10,     3000 },

    //真正写入外部设备的函数，比如设置继电器让方向舵切换左右转
    //  { SCHED_TASK(set_device_rc_out),                                                    100,     100 },
    //{ SCHED_TASK(write_device_II2C),                                                    1,    1000 },
    { SCHED_TASK(motors_output),                                                    1,    1000 },

    // 自驾仪虚拟地获取传感器数据，从all_external_device_input虚拟获取
    { SCHED_TASK(update_GPS),                                                  10,     100 },
    { SCHED_TASK(update_IMU),                                                  10,     100 },

    //自驾仪虚拟地输出数据，把控制量啥的输出到all_external_device_output
    //{ SCHED_TASK(update_external_device),                                                  10,     100 },

    { SCHED_TASK(get_gcs_udp),                                                    10,    1000 },
    { SCHED_TASK(send_ap2gcs_realtime_data_boatlink_by_udp),    1,    1000 },

    { SCHED_TASK(get_timedata_now),                                     1,     1000 },
    { SCHED_TASK(loop_one_second),                                      1,    10000 },

    //      { SCHED_TASK(record_log),                                                   100,    1100 },
    //      { SCHED_TASK(record_wp),                                                   100,    1100 },
    //      { SCHED_TASK(record_config),                                                   100,    1100 },

    { SCHED_TASK(end_of_task),                                          1000,    100 }
};

#define MAINTASK_TICK_TIME_MS 10//这个设置为10ms，对应每个循环100hz
int seconds = 0;
int micro_seconds = MAINTASK_TICK_TIME_MS * (1e3);/*每个tick对应的微秒数*/
struct timeval maintask_tick;
int main(int argc,char * const argv[])
{
    DEBUG_PRINTF("Welcome to BitPilot \n");

    // 初始化任务调度表
    boat.scheduler.init(&boat.scheduler_tasks[0], sizeof(boat.scheduler_tasks)/sizeof(boat.scheduler_tasks[0]));
    DEBUG_PRINTF(" sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]) = %d\n",sizeof(boat.scheduler_tasks)/sizeof(boat.scheduler_tasks[0]));

    //初始化步骤，初始化一些设备或者参数等
    boat.setup();

    while (1)
    {
        maintask_tick.tv_sec = seconds;
        maintask_tick.tv_usec = micro_seconds;
        select(0, NULL, NULL, NULL, &maintask_tick);

        boat.loop();
    }

    return 0;
}

void Boat::loop( void )
{
    uint32_t timer = (uint32_t)gettimeofday_us();//当前系统运行时间精确到微秒

    loop_fast();//在无人机中是姿态控制内环，在无人船中是制导控制环

    // 告诉调度器scheduler一个tick已经过去了，目前1个tick指的是10毫秒
    scheduler.tick();

    /*
     * loop_us这个应该是一个tick循环所指定的时间，比如我这里目前定义的是10ms，
     * 但是如果所有任务的执行时间的总和还是小于10ms呢，如果没有select这个定时器，就会一直循环
     * 这样就会导致scheduler_tasks数组中指定的频率失去原本的意义，所以必须有select定时器或者
     * 如果在单片机中，则使用某一个定时器来触发这个loop这个函数
     */
    uint32_t loop_us = micro_seconds;
    uint32_t time_available = loop_us - ( (uint32_t)gettimeofday_us() - timer );

    scheduler.run(time_available > loop_us ? 0u : time_available);
}

void Boat::loop_fast()
{
    fastloop_cnt ++;

    /*
     * 如果全部传感器都是硬件在环的，那么这里就不需要这个update_all_external_device_input
     * 如果传感器都硬件在环，那么就在数据有更新的时候，把数据填在这个结构中
     * 相当于在传感器和控制循环中间又添加了一层
     */
    update_all_external_device_input();

    /*1. decode_gcs2ap_radio*/
    decode_gcs2ap_udp();
#ifdef TEST
    gcs2ap_all_udp.rud_p = 2.0;
    gcs2ap_all_udp.rud_i = 0.0;
    gcs2ap_all_udp.rud_d = 0.0;
    gcs2ap_all_udp.cte_p = 2.0;
    gcs2ap_all_udp.cte_i = 0.0;
    gcs2ap_all_udp.cte_d = 0.0;
    gcs2ap_all_udp.arrive_radius = 50;
    gcs2ap_all_udp.cruise_throttle_percent = 100;
    gcs2ap_all_udp.workmode = AUTO_MODE;
    gcs2ap_all_udp.auto_workmode = AUTO_MISSION_MODE;
#endif

    /*2. navigation*/
    if( ! (fastloop_cnt % 100) )
    {
        navigation_loop();
    }

    global_bool_boatpilot.current_to_target_radian = (short)(auto_navigation.out_current_to_target_radian * 100.0);
    global_bool_boatpilot.current_to_target_degree = (short)(auto_navigation.out_current_to_target_degree * 100);
    global_bool_boatpilot.command_course_radian = (short)(auto_navigation.out_command_course_radian);
    global_bool_boatpilot.command_course_degree = (short)(auto_navigation.out_command_course_degree * 100);
    global_bool_boatpilot.wp_next = auto_navigation.out_current_target_wp_cnt;

    /*3 control*/
    control_loop();

    arm_motros_check();
    //motors_output();

    /*
     * 下面是把驾驶仪计算得到的电机或者舵机的输出给到simulator模拟器中
     */
    update_sim_water_craft();
}










