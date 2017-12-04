/*
 * all_external_device.h
 *
 *  Created on: 2017-9-18
 *      Author: wangbo
 */

#ifndef ALL_EXTERNAL_DEVICE_H_
#define ALL_EXTERNAL_DEVICE_H_



typedef struct tagAll_EXTERNAL_DEVICE_INPUT
{
	//gps
	float longitude;//度
	float latitude;//度
	float altitude;//米
	float v_north;//米每秒
	float v_east;//米每秒
	float v_down;//米每秒
	float heading;//度

	//imu
	float _gyro_x;//弧度每秒
	float _gyro_y;//弧度每秒
	float _gyro_z;//弧度每秒
	float _accel_x;//米每二次方秒
	float _accel_y;//米每二次方秒
	float _accel_z;//米每二次方秒

	//rc 遥控器信号输入
	float rc_raw_in_0;//1000～2000
	float rc_raw_in_1;
	float rc_raw_in_2;
	float rc_raw_in_3;
	float rc_raw_in_4;
	float rc_raw_in_5;
	float rc_raw_in_6;
	float rc_raw_in_7;
	float rc_raw_in_8;

	//姿态角
	float phi;
	float theta;
	float psi;;
}T_ALL_EXTERNAL_DEVICE_INPUT;

typedef struct tagAll_EXTERNAL_DEVICE_OUTPUT
{

	float rc_raw_out_0;//这个给到ucos系统中的pwm任务，输出pwm波
	float rc_raw_out_1;//这个给到ucos系统中的pwm任务，输出pwm波
	float rc_raw_out_2;//这个给到ucos系统中的pwm任务，输出pwm波
	float rc_raw_out_3;//这个给到ucos系统中的pwm任务，输出pwm波
	float rc_raw_out_4;//这个给到ucos系统中的pwm任务，输出pwm波
	float rc_raw_out_5;//这个给到ucos系统中的pwm任务，输出pwm波
	float rc_raw_out_6;//这个给到ucos系统中的pwm任务，输出pwm波
	float rc_raw_out_7;//这个给到ucos系统中的pwm任务，输出pwm波
	float rc_raw_out_8;//这个给到ucos系统中的pwm任务，输出pwm波

}T_All_EXTERNAL_DEVICE_OUTPUT;


extern T_ALL_EXTERNAL_DEVICE_INPUT all_external_device_input;//飞控本身自成体系，但是如果实际要用，则需要从外部获取传感器的数据，就从这个结构中取得，而且这里面的数据的单位都是以计算所需要的单位大小，比如经度是放大了10的7次方倍的

extern 	T_All_EXTERNAL_DEVICE_OUTPUT all_external_device_output;//飞控本身自成体系，但是如果实际要用，则需要把一些计算结果输出给外部设备，从这个结构中输出



#endif /* ALL_EXTERNAL_DEVICE_H_ */
