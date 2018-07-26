/*
 *@File     : BitBoat.cpp
 *@Author   : wangbo
 *@Date     : Nov 4, 2017
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */

#include "Boat.h"

/*
 * 这是任务调度表，除了fast_loop中的任务，其他任务都在这里执行，任务最多255个
 * 第1个字段是task的函数名称，第2个字段是该函数执行频率[hz]，第3个字段是分配给该函数的最大可执行时间[us]
 * 第2个字段执行频率，频率必然是小于scheduler的_loop_rate_hz，所以第2个字段执行频率，最小是1hz，最大是_loop_rate_hz，目前_loop_rate_hz=100
 * 第3个字段的数字是最大允许的时间，单位是[微秒]，范围是1~65535[us]
 * 每个任务都需要测试一下实际需要多长时间，如果大于每个tick所允许的剩余时间，就会出现Scheduler overrun task，剩余任务也就不会再执行
 * 因此需要特别注意串口读取 以及 udp读取等阻塞式的读取方式，阻塞等待的时间需要小于这个任务调度允许的最大时间
 * gps_Y901我给的最大允许是3000微秒，而gps_Y901是在10hz循环中的，因此该10hz循环中最多能有3个gps_Y901这种
 * 读取函数，否则就会把10ms全部占用了
 */
#define SCHED_TASK(func) (void (*)())&Boat::func

const BIT_Scheduler::Task Boat::scheduler_tasks[] =
{
    //真正读取传感器函数
    { SCHED_TASK(read_device_gps_JY901),                                        1,      MAX_WAIT_TIME_US_Y901 + 200 },
    { SCHED_TASK(read_device_gps_UM220),                                        1,      MAX_WAIT_TIME_US_UM220 + 200 },
    { SCHED_TASK(read_device_IMU_mpu6050),                                     10,      200 },

    //真正写入外部设备的函数，比如设置继电器让方向舵切换左右转
    { SCHED_TASK(write_device_motors_output),                                  20,     1000 },

    // 自驾仪虚拟地获取传感器数据，从all_external_device_input虚拟获取
    { SCHED_TASK(update_GPS),                                                  10,      200 },
    { SCHED_TASK(update_IMU),                                                  10,      200 },

    //自驾仪虚拟地输出数据，把控制量啥的输出到all_external_device_output
    //{ SCHED_TASK(update_external_device),                                    10,      100 },

    { SCHED_TASK(get_gcs_udp),                                                 10,      MAX_WAIT_TIME_US_UDP + 200 },
    { SCHED_TASK(update_navigation_loop),                                      10,      250 },
    { SCHED_TASK(send_ap2gcs_realtime_data_boatlink_by_udp),                    1,      350 },

    { SCHED_TASK(loop_1hz),                                                     1,      200 },

    { SCHED_TASK(record_log),                                                   1,      200 },
    { SCHED_TASK(record_wp),                                                    1,      200 },
    { SCHED_TASK(record_config),                                                1,      200 },

    { SCHED_TASK(end_of_task),                                                  1,       10 },
    { SCHED_TASK(get_timedata_now),                                             1,      200 }
};

int main(int argc,char * const argv[])
{
    DEBUG_PRINTF("Welcome to BitPilot \n");

    boat.scheduler.init(&boat.scheduler_tasks[0], sizeof(boat.scheduler_tasks)/sizeof(boat.scheduler_tasks[0])); // 初始化任务调度表
    DEBUG_PRINTF("There are %d tasks to run !!! \n",sizeof(boat.scheduler_tasks) / sizeof(boat.scheduler_tasks[0]));

    boat.setup(); // 初始化步骤，初始化一些设备或者参数等

    while (1)
    {
        boat.loop();
    }

    return 0;
}

void Boat::loop( void )
{
    //DEBUG_PRINTF("time_available_usec = %d \n", scheduler.time_available_usec());
    wait_us(scheduler.time_available_usec());

    uint64_t timer = (uint64_t)clock_us(); //当前系统运行时间精确到微秒

    loop_fast(); //在无人机中是姿态控制内环，在无人船中是制导控制环

    scheduler.tick(); // 告诉调度器scheduler一个tick已经过去了，目前1个tick指的是10毫秒

    /*
     * loop_us这个应该是一个tick循环所指定的时间，比如我这里目前定义的是10ms，
     * 但是如果所有任务的执行时间的总和还是小于10ms呢，如果没有select这个定时器，就会一直循环
     * 这样就会导致scheduler_tasks数组中指定的频率失去原本的意义，所以必须有select定时器或者
     * 如果在单片机中，则使用某一个定时器来触发这个loop这个函数，或者利用gettimeofday_ms函数把最小时间差缩短为1ms，那么每次tick至少就是1ms
     * clock_us() - timer 表示从loop开始到scheduler.run之前瞬间已经使用的时间
     */
    uint32_t loop_us = (uint32_t)(US_PER_SECOND / scheduler.get_loop_rate_hz()); // uint16 65535
    uint32_t time_available = loop_us - (uint32_t)( (uint64_t)clock_us() - (uint64_t)timer );

    //DEBUG_PRINTF("loop_fast   : time_available = %d \n", time_available);
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

    decode_gcs2ap_udp();

    control_loop();

    motros_arm_check();
    motors_set();

#if SIMULATE_BOAT
    /*
     * 把驾驶仪计算得到的电机或者舵机的输出给到simulator模拟器中
     */
    update_sim_water_craft();
#endif
}






