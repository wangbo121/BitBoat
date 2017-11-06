/*
 * BitBoat.cpp
 *
 *  Created on: Nov 4, 2017
 *      Author: wangbo
 */

#include "Boat.h"

#define MAINTASK_TICK_TIME_MS 10//这个设置为10ms，对应每个循环100hz
int seconds=0;
int micro_seconds=MAINTASK_TICK_TIME_MS*(1e3);/*每个tick为20毫秒，也就是20000微秒*/
struct timeval maintask_tick;

int main(int argc,char * const argv[])
{
    DEBUG_PRINTF("Welcome to BitPilot\n");

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
         */
        boat.loop();
    }

    return 0;
}
