/*

 * IMU.h
 *
 *  Created on: Jun 5, 2018
 *      Author: wangbo
 */
/*
 *@File     : IMU.h
 *@Author   : wangbo
 *@Date     : Jun 5, 2018
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */
#ifndef LIBRARIES_IMU_H_
#define LIBRARIES_IMU_H_

/*
 * Inertial Measure Unit
 * 惯性测量单元
 */

#include <stdint.h>


struct T_IMU_DATA
{
    /*
     * 这个是抽象数据结构，也就是关于IMU的数据都放在这个结构里面
     * 这个结构中的数据范围是由控制精度需求决定的
     * 比如姿态控制精度是厘度级别的
     */
    int acc_x;
    int acc_y;
    int acc_z;

    int gyro_x;
    int gyro_y;
    int gyro_z;

    int roll;//单位[0.01度]，扩大了100倍，范围是[-180 ~ +180度]
    int pitch;//单位[0.01度]，扩大了100倍，范围是[-90 ~ +90度]
    int yaw; // 单位[0.01度]，扩大了100倍，范围是[-180 ~ +180]

};

/*
 * IMU_data这个变量就是gps.h文件对外的接口，
 * IMU能够确定的物理量比如经度纬度速度等都从这里获取
 */
extern struct T_IMU_DATA IMU_data;

int IMU_uart_init();

int write_IMU_data();

int read_IMU_data();

int IMU_uart_close();





#endif /* LIBRARIES_IMU_H_ */
