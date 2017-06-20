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

//Local Quarternion Values
geometry_msgs::Quaternion botQ;

void setBotMode( std::string mode, ros::NodeHandle &node );

void getBotQ( const sensor_msgs::Imu &msg );

double radsToDeg( double rad );

int getQuadrant( double angle );

int getDirection( int quad1, int quad2 );

int main(int argc, char** argv)
{
	//Initial ROS settings
	ros::init(argc, argv, "erle_rand_walker");
	ros::NodeHandle n;
	int rate = 10;
	ros::Rate r(rate);

	time_t moveTime;

	tf::Quaternion botQuat;

	double yaw;
	double yaw_degrees;
	double newDirection;

	bool quatNormalFlag = false;

	ros::Publisher rcOverridePub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1 );
	mavros_msgs::OverrideRCIn msg_override;

	ros::Subscriber sub = n.subscribe("/mavros/imu/data", 1000, &getBotQ);
	setBotMode( "MANUAL" , n );

	while(ros::ok())
	{
		newDirection = rand() % 360;

		yaw = tf::getYaw( botQ );
		yaw_degrees = radsToDeg( yaw );

		if( isnan(yaw_degrees) == 0 && !quatNormalFlag )
		{
			quatNormalFlag = true;
		}

		if( quatNormalFlag )
		{
			if( getDirection( getQuadrant(yaw_degrees) , getQuadrant(newDirection) ) == 1 ||
				(getDirection( getQuadrant(yaw_degrees) , getQuadrant(newDirection) ) == 2 && newDirection > yaw_degrees) )
			{
				ROS_INFO("TURNING RIGHT...");
				//Force wheels right
				msg_override.channels[0] = 1100;
				rcOverridePub.publish(msg_override);
				ros::spinOnce();
				while( (int)newDirection != (int)yaw_degrees )
				{
					yaw = tf::getYaw( botQ );
					yaw_degrees = radsToDeg( yaw );
					ROS_INFO_STREAM("BOT DEG: " << yaw_degrees << ", TARGET DEG: " << newDirection );
					//reverse
					msg_override.channels[2] = 1552;
					rcOverridePub.publish(msg_override);
					ros::spinOnce();
					r.sleep();
				}
				ROS_INFO("TURNING COMPLETED");
			}

			if( getDirection( getQuadrant(yaw_degrees) , getQuadrant(newDirection) ) == 0 ||
				(getDirection( getQuadrant(yaw_degrees) , getQuadrant(newDirection) ) == 2 && newDirection < yaw_degrees) )
			{
				ROS_INFO("TURNING LEFT...");
				//Force wheels left
				msg_override.channels[0] = 1900;
				rcOverridePub.publish(msg_override);
				ros::spinOnce();
				while( (int)newDirection != (int)yaw_degrees )
				{
					yaw = tf::getYaw( botQ );
					yaw_degrees = radsToDeg( yaw );
					ROS_INFO_STREAM("BOT DEG: " << yaw_degrees << ", TARGET DEG: " << newDirection );
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
		}

		ros::spinOnce();
	}

	return 0;
}

int getDirection( int quad1, int quad2 )
{
	ROS_INFO_STREAM("QUADRANT 1: " << quad1 << ", QUADRANT 2: " << quad2 );
	if(quad1 == 1)
	{
		if(quad2 == 4)
			return 1;
		else 
			return 0;
	}

	else if(quad1 == 2)
	{
		if(quad2 == 1)
			return 1;
		else
			return 0;
	}
	else if(quad1 == 3)
	{
		if(quad2 == 2)
			return 1;
		else
			return 0;
	}
	else if(quad1 == 4)
	{
		if(quad2 == 3)
			return 1;
		else
			return 0;
	}
	else if(quad1 == quad2)
		return 2;

	return 1;
}

int getQuadrant( double angle )
{
	double intpart;
	double decimalPart = modf((angle / 360), &intpart);

	if( 0.0 < decimalPart && decimalPart < 0.25 )
		return 1;
	else if( 0.25 < decimalPart && decimalPart < 0.5 )
		return 2;
	else if( 0.5 < decimalPart && decimalPart < 0.75 )
		return 3;
	else if( 0.75 < decimalPart && decimalPart < 1.0 )
		return 4;
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

