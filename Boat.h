/*
 * Boat.h
 *
 *  Created on: Nov 6, 2017
 *      Author: wangbo
 */

#ifndef BOAT_H_
#define BOAT_H_

//C标准头文件
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>//创建文件
#include <pthread.h>
#include <semaphore.h>
#include <sys/stat.h>

//C++标准头文件
#include <iostream>
#include <cmath>

#include "global.h"



//#include "maintask.h"
//#include "loopfast.h"
//#include "loopslow.h"
//#include "utilityfunctions.h"
//#include "save_data.h"
//#include "boatlink.h"
//#include "gps.h"
//#include "navigation.h"
//#include "uart.h"
//#include "servo.h"
//#include "control.h"
//#include "modbus_485.h"
//#include "modbus_relay_switch.h"
//#include "udp.h"
//#include "commu.h"
//#include "bd.h"
//#include "generator.h"
//#include "radio.h"
//#include "location.h"










// Auto Pilot modes
// ----------------
#define STABILIZE 0                     // hold level position
#define ACRO 1                          // rate control  比例控制，其实也就是纯手动控制
#define ALT_HOLD 2                      // AUTO control
#define AUTO 3                          // AUTO control
#define GUIDED 4                        // AUTO control
#define LOITER 5                        // Hold a single location
#define RTL 6                           // AUTO control
#define CIRCLE 7                        // AUTO control
#define POSITION 8                      // AUTO control
#define LAND 9                          // AUTO control
#define OF_LOITER 10            // Hold a single location using optical flow



class Boat
{
public:
    friend class GCS_MAVLINK;

    Boat(void)
    {
    /*
    * 在构造函数的开始就初始化一些内部变量
    */
    control_mode            = STABILIZE;
    }



    void setup();
    void loop();

    public:

    private:
    uint8_t control_mode;

};

extern Boat boat;


#endif /* BOAT_H_ */
