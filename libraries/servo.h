/*
 * servo.h
 *
 *  Created on: Jul 16, 2016
 *      Author: wangbo
 */

#ifndef HEADERS_SERVO_H_
#define HEADERS_SERVO_H_

/*
 * 舵机和电机的控制都在这个源文件里
 * 根据具体的硬件写合适的程序，该注释勿删除
 */

#define DEFAULT_DEVICE_NUM 1 //默认的推进器通道
#define SPARE_DEVICE_NUM   2 // 冗余的推进器通道

int set_motors_init();

int set_motor_on();

int set_motor_off();

/*
 * device_num is for redundance
 */
int set_throttle_left_right(float pwm_left, float pwm_right, int device_num);

int set_motor_forward();

int set_motor_backward();

#endif /* HEADERS_SERVO_H_ */







