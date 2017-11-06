/*
 * all_external_device.cpp
 *
 *  Created on: 2017-9-18
 *      Author: wangbo
 */

#include "all_external_device.h"

T_ALL_EXTERNAL_DEVICE_INPUT all_external_device_input;//飞控本身自成体系，但是如果实际要用，则需要从外部获取传感器的数据，就从这个结构中取得，而且这里面的数据的单位都是以计算所需要的单位大小，比如经度是放大了10的7次方倍的
T_All_EXTERNAL_DEVICE_OUTPUT all_external_device_output;//飞控本身自成体系，但是如果实际要用，则需要把一些计算结果输出给外部设备，从这个结构中输出

