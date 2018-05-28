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
 * 根据具体的硬件写合适的程序
 */

#define DEFAULT_RUDDER_NUM 1
#define SPARE_RUDDER_NUM 2

/*
 * 因为在计算的时候把-45～45 总共90度映射到 -1～+1
 * 并且2/90=0.022，所以每度对应的纯数学计算值为0.022
 */
#define RUDDER_DEAD_ZONE 0.022*3

#define DEFAULT_DEVICE_NUM 1 //默认的推进器通道
#define SPARE_DEVICE_NUM   2 // 冗余的推进器通道

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

struct T_GLOBAL_SERVO
{
    unsigned char rudder_is_moving;
    float start_time_ms;
};

/*
 * Function:       set_rudder
 * Description:  pwm范围为1000-2000，-45--+45度，转换为-1.0--+1.0
 *                        rudder_num 默认为1，备用的方向舵为2
 */
int set_rudder(float pwm,int rudder_num);
int set_rudder_off(); // 停止方向舵，方向舵不再动作

/*左右推进器的差控制*/
/*
 * device_num is for redundance
 */
int set_throttle_left_right(float pwm_left, float pwm_right, int device_num);

int set_motor_forward();
int set_motor_backward();
int set_motor_on();
int set_motor_off();

int set_left_motor_on();
int set_left_motor_off();

int set_right_motor_on();

int set_right_motor_off();

int set_left_motor_forward();
int set_left_motor_backward();
int set_right_motor_forward();
int set_right_motor_backward();

int start_third_small_motor();
int stop_third_small_motor();

/*******下面的作为参考函数名，没有具体实现***************************************************************/
/*
 * Function:       set_throttle
 * Description:  pwm范围为1000-2000，0--+90度，对应0--10伏，转换为0--+1.0
 *                        throttle_num 默认为1，备用的油门为2
 */
int set_throttle(int pwm,int throttle_num);

/*使得方向舵可以运行pulse_ms的时间，类似与脉冲控制*/
int set_left_rudder_pulse(float pulse_ms);
int set_right_rudder_pulse(float pulse_ms);

/*一直左舵，一直右舵，停止方向舵*/
int set_left_rudder();
int set_right_rudder();
int set_stop_rudder();

int set_rudder_dot_move(float pwm);

#endif /* HEADERS_SERVO_H_ */
