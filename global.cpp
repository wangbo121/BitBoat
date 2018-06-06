/*
 *@File     : global.cpp
 *@Author   : wangbo
 *@Date     : Apr 3, 2018
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */
#include "global.h"

struct WAY_POINT wp_data[MAX_WAYPOINT_NUM];

int fd_boatpilot_log;
int fd_waypoint;
int fd_config;
int fd_socket_generic;

struct T_GLOBAL_BOOL_BOATPILOT  global_bool_boatpilot;
struct T_BIT_LOG boatpilot_log;
//struct T_CONFIG                 boatpilot_config_previous;
//struct T_CONFIG                 boatpilot_config;
