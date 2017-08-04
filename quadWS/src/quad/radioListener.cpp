#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <csignal>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <mavros_msgs/RadioStatus.h>

void getRadData( const mavros_msgs::RadioStatus &msg );

int main( int argc, char** argv )
{
	//Initial ROS settings
	ros::init(argc, argv, "radio_listener");
	ros::NodeHandle n;

	ros::Subscriber radList = n.subscribe("/mavros/radio_status", 1000, getRadData);
	
	ros::spin();

	return 0;
}

void getRadData( const mavros_msgs::RadioStatus &msg )
{
	ROS_INFO_STREAM( 
		"RSSI: " << msg.rssi <<
		"\n REMRSSI: " << msg.remrssi <<
		"\n TXBUF: " << msg.txbuf <<
		"\n NOISE: " << msg.noise <<
		"\n REMNOISE: " << msg.remnoise <<
		"\n RXERRORS: " << msg.rxerrors <<
		"\n FIXED: " << msg.fixed
	);
}
