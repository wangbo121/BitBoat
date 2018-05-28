/*
 * II2C.cpp
 *
 *  Created on: May 28, 2018
 *      Author: wangbo
 */

#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/dir.h>
#include <linux/types.h>
#include <string.h>
#include <poll.h>
//#include <sys/types.h>
#include <termios.h>
#include <linux/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <semaphore.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>


#include <sys/ioctl.h>


#include "global.h"
#include "II2C.h"


int fd_i2c;

int II2C_init()
{


    //DI/DO by PCF8574
    fd_i2c = open(DEV_I2C, O_RDWR);
    if (fd_i2c < 0)
    {
        printf("Open DEV_I2C failed: %d\n", fd_i2c);
        return -1;
    }
    else
    {
        printf("DEV_I2C open ok!\n");
        return fd_i2c;
    }


    return 0;
}

int write_device_II2C(int chip, unsigned char *buf, unsigned int len)
{
    int ret;
    unsigned int _chip;

    if(_chip < 0)  return -1;

    _chip = (unsigned int)chip;

    if(ioctl(fd_i2c, I2C_SLAVE_FORCE, _chip) >= 0 )
    {
        ret = write(fd_i2c, buf, len);
    }
    else
    {
        ret = -3;
    }

    return 0;
}


int PFC8574_DO(int chip, unsigned char data)
{
    unsigned char _chip;
    int ret=0;

    ret = 1;

    unsigned char send_buf[1];

    send_buf[0] = data;

    write_device_II2C(chip, send_buf, 1);

    return ret;
}



int DAC7574_DA(int chip, int channel, float voltage)
{
    unsigned char _chip;
    unsigned char _send_buf[3];
    int ret=0;

    _chip = (unsigned char)chip;
    _send_buf[0] = (unsigned char)channel;
    _send_buf[1] = DAC7574_MSB_V(voltage);
    _send_buf[2] = DAC7574_LSB_V(voltage);

    ret = write_device_II2C(_chip, _send_buf, sizeof(_send_buf));

    return ret;
}










