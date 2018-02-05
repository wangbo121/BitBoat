/*
 * utility.h
 *
 *  Created on: 2017-8-1
 *      Author: wangbo
 */

#ifndef UTILITY_H_
#define UTILITY_H_

/*
 * 以下暂时都是在Linux系统下的实现
 * 如果换作其他设备要做相应改变
 */
#ifndef LINUX_OS
#define LINUX_OS
#endif

#ifdef LINUX_OS
/*
 * 延时函数，如果更换操作系统需要重新改写
 */
int delay_ms(int ms);
int sleep_ms(int ms);

/*
 * 获取系统时间，年月日时分秒等
 */
float gettimeofday_s();
float gettimeofday_ms();
float gettimeofday_us();
float diff_gettimeofday_value(float start,float end);

/*
 * 获取系统开启或者主程序运行开始到当前时刻的时间计数
 * 获取以系统启动瞬间为基准的时间，单位是秒，毫秒，微秒
 * 且在编译链接时需加上 -lrt ;因为在librt中实现了clock_gettime函数。
 */
float clock_gettime_s();//获取系统开启后（主程序开始运行）到当前时刻的时间计数[s]秒
float clock_gettime_ms();//获取系统开启后（主程序开始运行）到当前时刻的时间计数[ms]毫秒
float clock_gettime_us(); //获取系统开启后（主程序开始运行）到当前时刻的时间计数[us]微秒
#endif

#endif /* UTILITY_H_ */
