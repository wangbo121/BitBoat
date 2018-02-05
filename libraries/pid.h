/*
 * pid.h
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#ifndef HEADERS_PID_H_
#define HEADERS_PID_H_

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

/*
 * 计算得到目标航向和实际航向误差导致的pid控制器的控制量
 * 单位是弧度，是方向舵的控制两
 */
float get_pid(float error, float scaler,float _kp,float _ki,float _kd);

/*
 * 把目标航向和实际航向的误差的积分和微分都清零
 */
void reset_I();

/*
 * 计算得到偏航距的pid控制修正量，为什么说是修正
 * 因为这个偏航距本身是为了修正航向，使得尽快靠近航线
 */
float get_pid_cte(float error, float scaler,float _kp,float _ki,float _kd);

/*
 * 把误差的积分和微分都清零
 */
void reset_I_cte();

#endif /* HEADERS_PID_H_ */
