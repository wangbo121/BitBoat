/*
 *@File     : utility.h
 *@Author   : wangbo
 *@Date     : Aug 1, 2017
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include <stdint.h>

/*
 * 以下暂时都是在Linux系统下的实现
 * 如果换作其他设备要做相应改变
 * 在编译链接时需加上 -lrt
 */

/*
 * 延时函数，如果更换操作系统需要重新改写
 */
int delay_ms(int ms);
int delay_us(uint32_t us);
int sleep_ms(int ms);

/*
 * 获取系统时间
 * 从UTC(coordinated universal time)时间
 * 1970年1月1日00时00分00秒(也称为Linux系统的Epoch时间)到当前时刻的秒数
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
uint64_t clock_us();

#endif /* UTILITY_H_ */










