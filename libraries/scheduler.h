/*
 * scheduler.h
 *
 *  Created on: Nov 3, 2017
 *      Author: wangbo
 */

#ifndef LIBRARIES_SCHEDULER_H_
#define LIBRARIES_SCHEDULER_H_

/*
 * 这是任务调度
 * 在开始的时候要调用scheduler.init()，然后每次循环都调用scheduler.tick()，每次循环时间大概是10ms
 * 最后调用scheduler.run()，这个run函数中的参数时最大允许执行时间，也就是当这个允许的时间到了后，
 * 就需要返回了，不再执行任务
 */

#include <stdint.h>

class BIT_Scheduler
{
public:
    typedef void (*task_fn_t) (void);

    struct Task
    {
        task_fn_t function;
        uint16_t interval_ticks;// 需要经过interval_ticks个周期，才能执行该任务
        uint16_t max_time_micros;
    };

    //调度初始化，指定任务表的地址，明确有多少个任务
    void init(const Task *tasks, uint8_t num_tasks);

    // 每一次tick都会把计数加1
    void tick(void);

    /*
     * 每一次tick都会调用一次这个run函数去执行尚未完成的scheduler中的任务
     * 执行调度任务表中的任务，参数是最大允许时间
     * 时间单位是微秒
     * 这个会在最大允许时间内执行尽可能多的任务
     */
    void run(uint16_t time_available);

    // return the number of microseconds available for the current task
    uint16_t time_available_usec(void);

private:
    // progmem list of tasks to run
    const struct Task *_tasks;

    // number of tasks in _tasks list
    uint8_t _num_tasks;

    // number of 'ticks' that have passed (number of times that
    // tick() has been called
    uint16_t _tick_counter;

    // tick counter at the time we last ran each task
    uint16_t *_last_run;

    // number of microseconds allowed for the current task
    uint16_t _task_time_allowed;

    // the time in microseconds when the task started
    uint32_t _task_time_started;
};

#endif /* LIBRARIES_SCHEDULER_H_ */
