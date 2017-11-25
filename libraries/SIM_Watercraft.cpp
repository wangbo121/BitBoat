/*
 * SIM_Watercraft.cpp
 *
 *  Created on: 2017-11-25
 *      Author: wangbo
 */

#include "SIM_Watercraft.h"
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h> //#define	RAND_MAX	2147483647
#include <iostream>

Watercraft::Watercraft(const char *home_str, const char *frame_str)
{



	char *saveptr=NULL;
	char *s = strdup(home_str);
	char *lat_s = strtok_r(s, ",", &saveptr);
	char *lon_s = strtok_r(NULL, ",", &saveptr);
	char *alt_s = strtok_r(NULL, ",", &saveptr);
	char *yaw_s = strtok_r(NULL, ",", &saveptr);

	memset(&home, 0, sizeof(home));
	home.lat = atof(lat_s) * 1.0e7;
	home.lng = atof(lon_s) * 1.0e7;
	home.alt = atof(alt_s) * 1.0e2;
	location = home;
	ground_level = home.alt*0.01;//20170818已测试，当home.alt=0.0时，ground_level也是0，所以on_ground函数，只需要判断position.z是不是大于0，大于0则是到地下了

	//std::cout<<"location.lng="<<location.lng<<std::endl;
	//std::cout<<"location.lat="<<location.lat<<std::endl;
	//std::cout<<" ground_level="<< ground_level<<std::endl;//20170818已测试
}

