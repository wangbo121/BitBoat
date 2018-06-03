/*
 * control.h
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#ifndef HEADERS_CONTROL_H_
#define HEADERS_CONTROL_H_

struct CTRL_PARA
{
	float rudder_p;
	float rudder_i;
	float rudder_d;

	unsigned char cte_max_correct_degree;//单位[度] 利用偏航距离计算算得到的补偿的最大补偿航向角 补偿目标course angle
    unsigned char cruise_throttle;

    /*设置标定，映射遥控器的输入*/
	unsigned char mmotor_on_pos;
	unsigned char mmotor_off_pos;
	unsigned char mmotor_onoff_mid;

	unsigned char mmotor_fwd_pos;
	unsigned char mmotor_bwd_pos;
	unsigned char mmotor_fwdbwd_mid;

	unsigned char rudder_left_pos;
	unsigned char rudder_right_pos;
	unsigned char rudder_mid_pos;

	unsigned char work_mode;
	unsigned char throttle_change_time_s; // [s] 自动驾驶时有效，作为改动百分之一油门量所需要的时间，单位是秒
};

struct T_PID
{
	float p;
	float i;
	float d;
};

struct CTRL_INPUT
{
	/*这是电台命令包传来的命令，用于手控控制和切换自动驾驶*/
	unsigned char ap_rc;
	unsigned char cmd;
	unsigned char test;
	unsigned char mmotor_bwd;
	unsigned char mmotor_on;

	float mmotor_onoff_pwm;
	float rudder_pwm;
	float mmotor_fwdbwd_pwm;

	/*下面的姿态，gps导航数据，航点编号都是为自动驾驶所准备的*/
	short pitch;
	short roll;
	short yaw;

	int gps_lng;/*[度*0.00001]*/
	int gps_lat;/*[度*0.00001]*/
	int gps_spd;/*[knot*0.01]*/
	int gps_dir;/*[度*0.01]*/

	int nextWP_lng;
	int nextWP_lat;
	unsigned char nextWP_no;
	unsigned char nextActWP_no;

	int home_lng;
	int home_lat;

	unsigned char rudder_reverse;
	unsigned char throttle_reverse;

	struct T_PID pid;

	float command_heading_angle_radian;//单位[弧度] 期望的 船头船尾 角度
    float gps_heading_angle_radian;//单位[弧度] 期望的 船头船尾 角度
    float command_course_angle_radian;//单位[弧度] 期望的 速度航向 角度
    float gps_course_angle_radian;//单位[弧度] 实际的 速度航向 角度
};

struct CTRL_OUTPUT
{
	float mmotor_onoff_pwm;
	float rudder_pwm;
	float mmotor_fwdbwd_pwm;

	float rudder_pwm_normalize;
	float throttle_pwm_normalize;

	float left_motor_pwm;
	float right_motor_pwm;

};

extern struct CTRL_PARA ctrlpara;
extern struct CTRL_INPUT ctrlinput;
extern struct CTRL_OUTPUT ctrloutput;

/*
 * Function:       control
 * Description:  利用ctrlinput，navigation，ctrlpara，gpsdata的数据，
 *                        计算方向舵和油门的输出量
 */
int control_loop(void);

/*
 * Function:       execute_ctrloutput
 * Description:  利用ctrlinput，navigation，ctrlpara，gpsdata的数据，
 *                        control_loop 只是计算出控制的量，但是并不输出，execute_ctrloutput才是输出
 *                        二者的频率可以相同，也可以不同
 */
int execute_ctrloutput(struct CTRL_OUTPUT *ptr_ctrloutput);

#endif /* HEADERS_CONTROL_H_ */
