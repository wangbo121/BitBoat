/*
 * servo.c
 *
 *  Created on: Jul 16, 2016
 *      Author: wangbo
 */

#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "utility.h"
#include "II2C.h"
#include "BIT_Math.h"

#include "servo.h"

/*目前油门最大电压为5伏特*/
#define MAX_THROTTLE 5

/*
 * 目前继电器就用到了第一个芯片，每一个芯片是8路继电器
 * 第1个继电器控制使用芯片1的DA1 和 DA2 输出 还是用第2个芯片的DA1 和 DA2  初始状态：常闭-使用的芯片1的  所以开机不需要动作 置0
 * 第2个继电器控制使用DA1 和 DA2 输出的通断                            初始状态：常开的-不输出电压   所以开机需要动作一下继电器 置1
 * 第3个继电器控制使用芯片1的DA3 和 DA4 输出 还是用第2个芯片的DA3 和 DA4  初始状态：常闭 默认使用的芯片1的
 * 第4个继电器控制使用DA3 和 DA4 输出的通断                            初始状态：常开的-不输出电压    所以开机需要动作一下继电器 置1
 * 第5个继电器控制 左 电机的正反装                                    初始状态：常闭的-是反转     所以开机需要动作一下继电器 置1
 * 第6个继电器控制 右 电机的正反装                                    初始状态：常闭的-是反转     所以开机需要动作一下继电器 置1
 * 所以模式初始状态是0x3a--0011 1010表示 电机都是正转 芯片1的 DA1 2 3 4都可以输出电压
 */
int set_motors_init()
{
    unsigned char send;

    send = 0x3a; // 整转 输出电压
    //send = 0x30; // 整转 不输出电压
    PFC8574_DO(PCF8574_DO1_ADDR, send);

    return 0;
}

int set_motor_on()
{
    unsigned char send;

    send = 0x3a; // 整转 输出电压
    PFC8574_DO(PCF8574_DO1_ADDR, send);

    return 0;
}

int set_motor_off()
{
    unsigned char send;

    send = 0x30; // 整转 不输出电压
    PFC8574_DO(PCF8574_DO1_ADDR, send);

    return 0;
}


int set_motors_speed(float *motors_speed)
{
    motors_speed[0] = constrain_value(motors_speed[0], 1000.0f, 2000.0f);
    motors_speed[1] = constrain_value(motors_speed[1], 1000.0f, 2000.0f);

    float pwm_left_normalize=0.0;
    float pwm_right_normalize=0.0;

    float voltage;

    /*pwm within 1000-2000 to 0-1000 to 0--+1*/
    pwm_left_normalize  = ((float)motors_speed[0] - 1000) / 1000;
    pwm_right_normalize = ((float)motors_speed[1] - 1000) / 1000;

    voltage = pwm_left_normalize * MAX_THROTTLE;
    DAC7574_DA(DAC7574_DA1_ADDR, DA_CHANNEL_1, voltage); // left motor

    voltage = pwm_right_normalize * MAX_THROTTLE;
    DAC7574_DA(DAC7574_DA1_ADDR, DA_CHANNEL_0, voltage); // right motor

    return 0;
}



int set_throttle_left_right(float pwm_left, float pwm_right, int device_num)
{
    float pwm_left_normalize=0.0;
    float pwm_right_normalize=0.0;

    if(pwm_left < 1000.0)
    {
        pwm_left = 1000.0;
    }
    else if(pwm_left > 2000.0)
    {
        pwm_left = 2000.0;
    }

    if(pwm_right < 1000.0)
    {
        pwm_right = 1000.0;
    }
    else if(pwm_right > 2000.0)
    {
        pwm_right = 2000.0;
    }

    /*pwm within 1000-2000 to 0-1000 to 0--+1*/
    pwm_left_normalize=((float)pwm_left-1000)/1000;
    pwm_right_normalize=((float)pwm_right-1000)/1000;

    float voltage;
    switch(device_num)
    {
    case DEFAULT_DEVICE_NUM:
        voltage = pwm_left_normalize * MAX_THROTTLE;
        DAC7574_DA(DAC7574_DA1_ADDR, DA_CHANNEL_1, voltage); // left motor

        voltage = pwm_right_normalize * MAX_THROTTLE;
        DAC7574_DA(DAC7574_DA1_ADDR, DA_CHANNEL_0, voltage); // right motor
        break;
    case SPARE_DEVICE_NUM:
        break;
    default:
        break;
    }

    return 0;
}

int set_motor_forward()
{

    return 0;
}

int set_motor_backward()
{

    return 0;
}





