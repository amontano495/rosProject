#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <csignal>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

//Local Quarternion Values
geometry_msgs::Quaternion botQ;

void setBotMode( std::string mode, ros::NodeHandle &node );

void getBotQ( const sensor_msgs::Imu &msg );

double radsToDeg( double rad );

int main(int argc, char** argv)
{
	//Initial ROS settings
	ros::init(argc, argv, "erle_rand_walker");
	ros::NodeHandle n;

	time_t moveTime;

	tf::Quaternion botQuat;

	double yaw;
	double yaw_degrees;
	double newDirection;

	ros::Subscriber sub = n.subscribe("/mavros/imu/data", 1000, &getBotQ);
	setBotMode( "MANUAL" , n );

	while(ros::ok())
	{
		yaw = tf::getYaw( botQ );
		yaw_degrees = radsToDeg( yaw );

		newDirection = rand() % 360;
		ROS_INFO_STREAM("NEW DIRECTION: " << newDirection );

		if( newDirection > 0 && newDirection < 180 )
		{
			ROS_INFO("TURNING...");
			//Force wheels right
			while( (int)newDirection != (int)yaw_degrees )
			{
				ROS_INFO_STREAM("BOT DEG: " << yaw_degrees << ", TARGET DEG: " << newDirection );
				//reverse
			}
			ROS_INFO("TURNING COMPLETED");
		}

		if( newDirection > 180 && newDirection < 360 )
		{
			ROS_INFO("TURNING...");
			//Force wheels left
			while( (int)newDirection != (int)yaw_degrees )
			{
				ROS_INFO_STREAM("BOT DEG: " << yaw_degrees << ", TARGET DEG: " << newDirection );
				//reverse
			}
			ROS_INFO("TURNING COMPLETED");
		}

		ROS_INFO("MOVING...");
		moveTime = time(NULL) + 5;
		while( time(NULL) < moveTime )
		{
			//forward
		}	
		ros::spinOnce();
	}

	return 0;
}

double radsToDeg( double rad )
{
	double deg = rad * 180 / M_PI;
	if( deg < 0 )
		deg += 360;
	return deg;
}

void getBotQ( const sensor_msgs::Imu &msg )
{
	botQ = msg.orientation;

}

void setBotMode( std::string mode, ros::NodeHandle &node)
{
	mavros_msgs::SetMode setMode;
	mavros_msgs::State currentState;
	ros::ServiceClient setModeClient = node.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	setMode.request.custom_mode = mode;

	if( currentState.mode != mode )
	{
		if( setModeClient.call(setMode) )
			ROS_INFO_STREAM("MODE UPDATED TO: " << mode);
		else
			ROS_ERROR_STREAM("FAILED TO UPDATE MODE TO: " << mode);
	}
}

