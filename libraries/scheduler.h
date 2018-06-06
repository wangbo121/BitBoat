/*
 *@File     : scheduler.h
 *@Author   : wangbo
 *@Date     : Nov 3, 2017
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
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
        uint16_t  rate_hz;
        uint16_t  max_time_micros; // 目前用uint16_t表示，也就是最多是65毫秒，如果串口波特率是9600的，这个指的是传输速率，并不是读取速率，读取速率比传输速率要快很多
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
    void run(uint32_t time_available);

    // return the number of microseconds available for the current task
    uint32_t time_available_usec(void);

    uint16_t get_loop_rate_hz(void);

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
    uint64_t _task_time_started;

    uint16_t _loop_rate_hz; // fastest freq is 1000hz

    uint32_t _spare_micros;
};

#endif /* LIBRARIES_SCHEDULER_H_ */
