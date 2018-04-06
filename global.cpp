/*
 * global.cpp
 *
 *  Created on: 2018-4-3
 *      Author: wangbo
 */


#include "global.h"

struct WAY_POINT wp_data[MAX_WAYPOINT_NUM];

int fd_boatpilot_log;
int fd_waypoint;
int fd_config;
int fd_socket_generic;

struct T_GLOBAL_BOOL_BOATPILOT  global_bool_boatpilot;
struct T_BIT_LOG boatpilot_log;
