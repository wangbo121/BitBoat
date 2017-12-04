/*
 * scheduler.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: wangbo
 */

#include <stdio.h>
#include <string.h>
#include "scheduler.h"
#include "utility.h"


// initialise the scheduler
void AP_Scheduler::init(const AP_Scheduler::Task *tasks, uint8_t num_tasks)
{
    _tasks = tasks;
    _num_tasks = num_tasks;
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _tick_counter = 0;
}

// one tick has passed
void AP_Scheduler::tick(void)
{
    _tick_counter++;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void AP_Scheduler::run(uint16_t time_available)
{
    uint16_t interval_ticks;

//    printf("hello run %d\n",time_available);
//
//               return ;

    for (uint8_t i=0; i<_num_tasks; i++) {
        uint16_t dt = _tick_counter - _last_run[i];
        //uint16_t interval_ticks = (&_tasks[i].interval_ticks);
        memcpy(&interval_ticks,&_tasks[i].interval_ticks,sizeof(uint16_t));

       //printf("hello run %d\n",interval_ticks);
//
//             return ;

        if (dt >= interval_ticks) {
            // this task is due to run. Do we have enough time to run it?
            //_task_time_allowed = (&_tasks[i].max_time_micros);
            memcpy(&_task_time_allowed,&_tasks[i].max_time_micros,sizeof(uint16_t));



            if (dt >= interval_ticks*2) {
                printf("Scheduler slip task[%u] \n",(unsigned)i);
                // we've slipped a whole run of this task!
//                if (_debug != 0) {
//                    hal.console->printf_P(PSTR("Scheduler slip task[%u] (%u/%u/%u)\n"),
//                                          (unsigned)i,
//                                          (unsigned)dt,
//                                          (unsigned)interval_ticks,
//                                          (unsigned)_task_time_allowed);
//                }
            }

            if (_task_time_allowed <= time_available) {
                // run it
                //_task_time_started = hal.scheduler->micros();


//                printf("hello run %d\n",_task_time_allowed);
//
//                                                        return ;

                _task_time_started = (uint32_t)gettimeofday_us();
//                task_fn_t func = (task_fn_t)(&_tasks[i].function);
//                func();
                _tasks[i].function();




                // record the tick counter when we ran. This drives
                // when we next run the event
                _last_run[i] = _tick_counter;

                // work out how long the event actually took
                uint32_t time_taken = (uint32_t)gettimeofday_us() - _task_time_started;

                if (time_taken > _task_time_allowed) {
                    printf("Scheduler overrun task[%u]\n",(unsigned)i);
                    // the event overran!
//                    if (_debug > 1) {
//                        hal.console->printf_P(PSTR("Scheduler overrun task[%u] (%u/%u)\n"),
//                                              (unsigned)i,
//                                              (unsigned)time_taken,
//                                              (unsigned)_task_time_allowed);
//                    }
                    return;
                }
                time_available -= time_taken;
            }
        }
    }
}
