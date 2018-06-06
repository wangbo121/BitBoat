/*
 *@File     : scheduler.cpp
 *@Author   : wangbo
 *@Date     : Nov 3, 2017
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */
#include <stdio.h>
#include <string.h>

#include "utility.h" // 获取时间us ms s函数

#include "scheduler.h"

// 初始化调度任务表及其参数
void BIT_Scheduler::init(const BIT_Scheduler::Task *tasks, uint8_t num_tasks)
{
    _tasks = tasks;
    _num_tasks = num_tasks;  // 因为_num_tasks是uint8_t的 所以任务最多有255个
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _tick_counter = 0;     // 因为主循环最大是1000hz 而最低频率在run函数中限制为1hz 所以_tick_counter的范围是1 ~ 1000，在uint16_t范围内
    _loop_rate_hz = 100; // 100hz 主循环是0.01s 一次 最大频率目前限制为1000hz
    //_loop_rate_hz = 1;     //   1hz 主循环是   1s 一次 最大频率目前限制为1000hz
}

// 经过了一个tick
void BIT_Scheduler::tick(void)
{
    _tick_counter++;
    //printf("_tick_counter = %d \n", _tick_counter);
}

/*
 * 执行一次tick
 * 这个会在最大允许时间内执行尽可能多的任务
 */
void BIT_Scheduler::run(uint32_t time_available)
{
    for (uint8_t i=0; i<_num_tasks; i++)
    {
        uint16_t dt = _tick_counter - _last_run[i];

        uint16_t interval_ticks = _loop_rate_hz / _tasks[i].rate_hz;
        if(interval_ticks < 1)
        {
            interval_ticks = 1;
        }

        if (dt >= interval_ticks)
        {
        	// 拷贝该第i个任务允许执行的时间
            memcpy(&_task_time_allowed, &_tasks[i].max_time_micros, sizeof(uint16_t));
            //printf("_tasks[%d].max_time_micros = %d \n", i, _tasks[i].max_time_micros);

            if (dt >= interval_ticks * 2)
            {
                printf("Scheduler slip task[%u] \n",(unsigned)i);// 说明应该执行该任务的但是错过了一次
            }

            if (_task_time_allowed <= time_available)
            {

                _task_time_started = (uint64_t)clock_us();
                _tasks[i].function();

                /*
                 * 记录该任务这一次执行时计数器tick的值
                 */
                _last_run[i] = _tick_counter;

                // 计算这个任务到底需要花了多少时间
                uint16_t time_taken = (uint64_t)clock_us() - _task_time_started;
                //printf("task [%2d]  time_taken    : =%d \n", i, time_taken);

                if (time_taken > _task_time_allowed)
                {
                    printf("Scheduler overrun task[%2u], time_taken is:%d \n", (unsigned)i, time_taken);
                    return;
                }
                if(time_taken >= time_available)
                {
                    //_spare_micros = 0;
                    _spare_micros = 10; // 按道理应该是不再等待的，因为这些任务加起来已经把所有的时间给用完了，但是为了保险起见我还是等待10us吧
                }
                time_available -= time_taken;
            }
        }
    } // for() end
    _spare_micros = time_available;
}

uint32_t BIT_Scheduler::time_available_usec(void)
{
    return _spare_micros;
}

uint16_t BIT_Scheduler::get_loop_rate_hz(void)
{
    if(_loop_rate_hz < 1)
    {
        return 1;
    }
    else if(_loop_rate_hz > 1000)
    {
        return 1000; // 最高1000hz
    }

    return _loop_rate_hz;
}












