/*
 * global.h
 *
 *  Created on: Nov 6, 2017
 *      Author: wangbo
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_


/**
 * 简单打印调试信息
 */
//#define DEBUG_SWITCH   1//如果不想打印信息，就将这句代码注释掉
#ifdef    DEBUG_SWITCH
//#define printf_debug(fmt,args...) printf(fmt, ##args)
#define DEBUG_PRINTF(fmt,args...) printf(fmt, ##args)
#else
#define DEBUG_PRINTF(fmt,args...) /*do nothing */
#endif

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))


#endif /* GLOBAL_H_ */
