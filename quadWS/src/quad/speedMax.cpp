#include <string>
#include <vector>
#include <csignal>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mavros_mission_push");

	ros::Time::init();

	int rate = 10;
	ros::Rate r(rate);

	ros::NodeHandle n;
	ros::ServiceClient pushClient;
	mavros_msgs::WaypointPush wayPusher;
	//Send waypoints to rover

	return 0;
}
