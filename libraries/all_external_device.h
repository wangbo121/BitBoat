/*
 * all_external_device.h
 *
 *  Created on: 2017-9-18
 *      Author: wangbo
 */

#ifndef ALL_EXTERNAL_DEVICE_H_
#define ALL_EXTERNAL_DEVICE_H_

/*
 * 我打算的是把驾驶仪自己独立成为一个模块
 * 而驾驶仪需要的所有的外部设备的信号，以及驾驶仪想往外部发送的信号都放在这里
 * 其实，比较理想的方式应该是px4的uorb模式，但是没有学会，就先这么用
 */

typedef struct TYPE_All_EXTERNAL_DEVICE_INPUT
{
    /*
     * 无论实际外部设备的数据是否是放大或者缩小后的
     * 这个结构中的数据都是具有实际物理意义的，并且保存的是国际单位，如果有正负的那就带正负号
     * 比如速度是      米每秒
     *    加速度是     米每二次方秒
     *    高度是       米
     */

	//gps
	float longitude;//度
	float latitude;//度
	float altitude;//米
	float course; // 度 速度方向
	float speed; // north east down 合成的速度 单位是米每秒
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

typedef struct TYPE_All_EXTERNAL_DEVICE_OUTPUT
{
	float rc_raw_out_0;//这个给到ucos系统中的pwm任务，输出pwm波
	float rc_raw_out_1;
	float rc_raw_out_2;
	float rc_raw_out_3;
	float rc_raw_out_4;
	float rc_raw_out_5;
	float rc_raw_out_6;
	float rc_raw_out_7;
	float rc_raw_out_8;
}T_All_EXTERNAL_DEVICE_OUTPUT;

/*
 * 飞控驾驶仪本身自成体系，本身是数学信号，但是如果实际要用，则需要从外部获取传感器的数据，
 * 就从这个结构中取得，而且这里面的数据的单位都是以计算所需要的单位大小，比如经度是放大了10的7次方倍的
 */
extern T_ALL_EXTERNAL_DEVICE_INPUT all_external_device_input;

/*
 * 飞控本身自成体系，但是如果实际要用，
 * 则需要把一些计算结果输出给外部设备，从这个结构中输出
 */
extern 	T_All_EXTERNAL_DEVICE_OUTPUT all_external_device_output;

#endif /* ALL_EXTERNAL_DEVICE_H_ */
