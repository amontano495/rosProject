#include <string>
#include <math.h>
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

void setBotMode( std::string mode, ros::NodeHandle &node );

int main(int argc, char** argv)
{
	//Initial ROS settings
	ros::init(argc, argv, "erle_rand_walker");
	ros::NodeHandle n;
	int rate = 10;
	ros::Rate r(rate);

	time_t moveTime;
	time_t revTime;

	bool quatNormalFlag = false;

	ros::Publisher rcOverridePub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1 );
	mavros_msgs::OverrideRCIn msg_override;

	setBotMode( "MANUAL" , n );

	while(ros::ok())
	{
		int randomDirection = rand() % 1;
		int randomTime = rand() % 3;

		if( randomDirection == 1 ) 
		{
			ROS_INFO("TURNING RIGHT...");
			//Force wheels right
			msg_override.channels[0] = 1100;
			rcOverridePub.publish(msg_override);
			ros::spinOnce();

			revTime = time(NULL) + randomTime;
			while( time(NULL) < revTime )
			{
				//reverse
				msg_override.channels[2] = 1552;
				rcOverridePub.publish(msg_override);
				ros::spinOnce();
				r.sleep();
			}
			ROS_INFO("TURNING COMPLETED");
		}

		else 
		{
			ROS_INFO("TURNING LEFT...");
			//Force wheels left
			msg_override.channels[0] = 1900;
			rcOverridePub.publish(msg_override);
			ros::spinOnce();

			revTime = time(NULL) + randomTime;
			while( time(NULL) < revTime )
			{
				//reverse
				msg_override.channels[2] = 1552;
				rcOverridePub.publish(msg_override);
				ros::spinOnce();
				r.sleep();
			}
			ROS_INFO("TURNING COMPLETED");
		}

		

		msg_override.channels[0] = 1500;
		rcOverridePub.publish(msg_override);
		ros::spinOnce();

		ROS_INFO("MOVING...");
		moveTime = time(NULL) + 2;
		while( time(NULL) < moveTime && quatNormalFlag )
		{
/*
			//forward
			msg_override.channels[2] = 1425;
			rcOverridePub.publish(msg_override);
			ros::spinOnce();
			r.sleep();
*/
		}	

		ros::spinOnce();
	}

	return 0;
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

