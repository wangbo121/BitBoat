/*
 * scheduler.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: wangbo
 */

#include <stdio.h>
#include <string.h>

#include "utility.h"

#include "scheduler.h"

// 初始化调度任务表及其参数
void BIT_Scheduler::init(const BIT_Scheduler::Task *tasks, uint8_t num_tasks)
{
    _tasks = tasks;
    _num_tasks = num_tasks;
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _tick_counter = 0;
}

// 经过了一个tick
void BIT_Scheduler::tick(void)
{
    _tick_counter++;
}

/*
 * 执行一次tick
 * 这个会在最大允许时间内执行尽可能多的任务
 */
void BIT_Scheduler::run(uint16_t time_available)
{
    uint16_t interval_ticks;

    for (uint8_t i=0; i<_num_tasks; i++)
    {
        uint16_t dt = _tick_counter - _last_run[i];
        memcpy(&interval_ticks,&_tasks[i].interval_ticks,sizeof(uint16_t));

        if (dt >= interval_ticks)
        {
        	// 拷贝该第i个任务允许执行的时间
            memcpy(&_task_time_allowed,&_tasks[i].max_time_micros,sizeof(uint16_t));

            if (dt >= interval_ticks*2)
            {
                printf("Scheduler slip task[%u] \n",(unsigned)i);// 说明应该执行该任务的但是错过了一次
            }

            if (_task_time_allowed <= time_available)
            {

                _task_time_started = (uint32_t)gettimeofday_us();
                _tasks[i].function();

                /*
                 * 记录该任务这一次执行时计数器tick的值
                 */
                _last_run[i] = _tick_counter;

                // 计算这个任务到底需要花了多少时间
                uint32_t time_taken = (uint32_t)gettimeofday_us() - _task_time_started;

                if (time_taken > _task_time_allowed)
                {
                    printf("Scheduler overrun task[%u]\n",(unsigned)i);
                    return;
                }
                time_available -= time_taken;
            }
        }
    }
}
