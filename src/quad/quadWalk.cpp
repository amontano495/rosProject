#include <string>
#include <math.h>
#include <random>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <csignal>
#include <ros/ros.h>
#include <boost/thread.hpp>
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
#include <sensor_msgs/NavSatFix.h>

#include "botLib.h"

#define REVERSE_SPEED 1400
#define FORWARD_SPEED 1600
#define ZERO_SPEED 1500
#define LEFT_TURN 1100
#define RIGHT_TURN 1900
#define STRAIGHT 1500
#define LEFT 0
#define RIGHT 1

int main(int argc, char** argv)
{
	//Initial ROS settings
	ros::init(argc, argv, "erle_rand_walker");
	ros::NodeHandle n;
	int rate = 10;
	ros::Rate r(rate);

	//times to move and reverse
	time_t moveTime;
	time_t revTime;
	time_t backTime;

	//randomized variables for nondeterministic movement
	int randomDirection;
	int randomTime;

	bool traveling = true;

	//Used to gain information from GPS device
	ros::Subscriber gpsSub = n.subscribe("/mavros/global_position/raw/fix", 1000, &getBotCoords );

	ros::Publisher rcPub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);


	//fixed time from user input
	int inputMoveTime = 20;

	//Represents vertices of a square
	coord quadPolygon[4];

	quadPolygon[0].lat = 39.539045;
	quadPolygon[0].lon = -119.814623;

	quadPolygon[1].lat = 39.537889;
	quadPolygon[1].lon = -119.813714;

	quadPolygon[2].lat = 39.539155;
	quadPolygon[2].lon = -119.814138;

	quadPolygon[3].lat = 39.537783;
	quadPolygon[3].lon = -119.814214;



	//Sets robot to "MANUAL" mode
	setBotMode( "MANUAL" , n );

	while(ros::ok())
	{
		//randomly pick a direction and time length
		randomDirection = rand() % 2;
		randomTime = getUniRand(1, 2);


		//spin backwards in order to pick a new, random direction
		if( randomDirection == RIGHT ) 
		{
			ROS_INFO("TURNING RIGHT...");

			revTime = time(NULL) + randomTime;
			while( time(NULL) < revTime )
			{
				//Force wheels right and reverse
				setBotMovement( REVERSE_SPEED, RIGHT_TURN, rcPub );
				r.sleep();
			}

			setBotMovement( ZERO_SPEED, STRAIGHT, rcPub );
			ROS_INFO("TURNING COMPLETED");
		}

		if( randomDirection == LEFT )
		{
			ROS_INFO("TURNING LEFT...");

			revTime = time(NULL) + randomTime;
			while( time(NULL) < revTime )
			{
				//Force wheels left
				setBotMovement( REVERSE_SPEED, LEFT_TURN, rcPub );
				r.sleep();
			}

			setBotMovement( ZERO_SPEED, STRAIGHT, rcPub );
			ROS_INFO("TURNING COMPLETED");
		}



		//move forward for fixed time or until out of bounds
		ROS_INFO("MOVING...");

		moveTime = time(NULL) + inputMoveTime;
		traveling = true;
		while( traveling )
		{
			//return wheels to forward position
			setBotMovement( FORWARD_SPEED, STRAIGHT, rcPub );

			//case: out of bounds
			if( !(boundaryCheck(quadPolygon)) )
			{
				ROS_INFO("HIT BOUNDARY");
				
				//force bot to move towards within polygon
				returnToBoundary( n );
				setBotMode( "AUTO" , n );
				while( !(boundaryCheck(quadPolygon)) )
				{
					ros::spinOnce();
					r.sleep();
				}
				setBotMode( "MANUAL" , n );

				setBotMovement( ZERO_SPEED, STRAIGHT, rcPub );
				traveling = false;
			}

			//case: time limit for moving has ended
			if( time(NULL) == moveTime )
			{
				ROS_INFO("MOVING COMPLETED");
				traveling = false;
			}

			ros::spinOnce();
			r.sleep();
		}

		ros::spinOnce();
	}

	return 0;
}
