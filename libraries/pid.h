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

float get_pid(float error, float scaler,float _kp,float _ki,float _kd);
void reset_I();

float get_pid_cte(float error, float scaler,float _kp,float _ki,float _kd);
void reset_I_cte();


#endif /* HEADERS_PID_H_ */
