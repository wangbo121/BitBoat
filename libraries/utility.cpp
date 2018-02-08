/*
 * utility.cpp
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include "utility.h"

#ifndef LINUX_OS
#define LINUX_OS
#endif

/*
 * 精确到毫秒的延时
 */
int delay_ms(int ms)
{
#ifdef LINUX_OS
	struct timeval delay;
	delay.tv_sec = 0;
	delay.tv_usec = ms * 1000;
	select(0, NULL, NULL, NULL, &delay);
#endif
	return 0;
}

int sleep_ms(int ms)
{
#ifdef LINUX_OS
	usleep(1000*ms);
#endif
	return 0;
}

/*
 * 获取系统时间
 * 从UTC(coordinated universal time)时间
 * 1970年1月1日00时00分00秒(也称为Linux系统的Epoch时间)到当前时刻的秒数
 */
float gettimeofday_s()
{
#ifdef LINUX_OS
	struct timeval current_time;

	gettimeofday(&current_time,NULL);

	return (float)(current_time.tv_sec)*1+(float)(current_time.tv_usec)*1e-6;
#else
	return 0;
#endif
}

float gettimeofday_ms()
{
#ifdef LINUX_OS
	struct timeval current_time;

	gettimeofday(&current_time,NULL);

	return (float)(current_time.tv_sec)*1e3+(float)(current_time.tv_usec)*1e-3;
#else
	return 0;
#endif
}

float gettimeofday_us()
{
#ifdef LINUX_OS
	struct timeval current_time;

	gettimeofday(&current_time,NULL);

	return (float)(current_time.tv_sec)*1e6+(float)(current_time.tv_usec)*1;
#else
	return 0;
#endif
}

float diff_gettimeofday_value(float start,float end)
{
	return end-start;
}

/*
 * 获取以系统启动瞬间为基准的时间
 * long sys_clock_gettime (clockid_t which_clock, struct timespec *tp);
 */
/*
 *  且在编译链接时需加上 -lrt ;因为在librt中实现了clock_gettime函数。
 *  struct timespec ts;
 *  clock_gettime(CLOCK_MONOTONIC,ts);
 */
float clock_gettime_s()
{
#ifdef LINUX_OS
	struct timespec current_time;

	clock_gettime(CLOCK_MONOTONIC, &current_time);

	return (float)(current_time.tv_sec)*1+(float)(current_time.tv_nsec)*1e-9;
#else
	return 0;
#endif
}

float clock_gettime_ms()
{
    float time_s=0.0;

    time_s=clock_gettime_s();

    return time_s*1e3;
}

float clock_gettime_us()
{
    float time_s=0.0;

    time_s=clock_gettime_s();

    return time_s*1e6;
}
