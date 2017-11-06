/*
 * scheduler.h
 *
 *  Created on: Nov 3, 2017
 *      Author: wangbo
 */

#ifndef LIBRARIES_SCHEDULER_H_
#define LIBRARIES_SCHEDULER_H_


/*
  A task scheduler for APM main loops

  Sketches should call scheduler.init() on startup, then call
  scheduler.tick() at regular intervals (typically every 10ms).

  To run tasks use scheduler.run(), passing the amount of time that
  the scheduler is allowed to use before it must return
 */

#include <stdint.h>

class AP_Scheduler
{
public:
    typedef void (*task_fn_t)(void);

    struct Task {
        task_fn_t function;
        uint16_t interval_ticks;
        uint16_t max_time_micros;
    };

    // initialise scheduler
    void init(const Task *tasks, uint8_t num_tasks);

    // call when one tick has passed
    void tick(void);

    // run the tasks. Call this once per 'tick'.
    // time_available is the amount of time available to run
    // tasks in microseconds
    void run(uint16_t time_available);

    // return the number of microseconds available for the current task
    uint16_t time_available_usec(void);

    // return debug parameter
    uint8_t debug(void) { return _debug; }

    //static const struct AP_Param::GroupInfo var_info[];

private:
    // used to enable scheduler debugging
    int8_t _debug;

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
