#ifndef BOTLIB_H
#define BOTLIB_H

#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

void setBotMode( std::string mode, ros::NodeHandle &node );

#endif
