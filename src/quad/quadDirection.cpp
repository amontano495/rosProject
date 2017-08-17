#include <string>
#include <math.h>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <csignal>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/WaypointPush.h>
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
	ros::init(argc, argv, "erle_rand_direction");
	ros::NodeHandle n;
	int rate = 10;
	ros::Rate r(rate);

	int inputMoveTime;

	//times to move and reverse
	time_t moveTime;
	time_t revTime;
	time_t stopTime;

	//randomized variables for nondeterministic movement
	int randomDirection;
	int randomTime;


	//Dictate the throttle and steering of the robot
	ros::Publisher rcPub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1 );

	//Used to gain information from GPS device
	ros::Subscriber gpsSub = n.subscribe("/mavros/global_position/global", 1000, &getBotCoords );

	coord quadPolygon[4];
	coord centerCoord;

	//Lat/Lon vertices of the polygon
	//Must be in order
	setPolyVertsFromFile( quadPolygon[0], quadPolygon[2], quadPolygon[1], quadPolygon[3], centerCoord);
/*
	//NW vertex
	quadPolygon[0].lat = 39.539045;
	quadPolygon[0].lon = -119.814623;

	//SW vertex
	quadPolygon[1].lat = 39.537889;
	quadPolygon[1].lon = -119.813714;

	//NE vertex
	quadPolygon[2].lat = 39.539155;
	quadPolygon[2].lon = -119.814138;

	//SE vertex
	quadPolygon[3].lat = 39.537783;
	quadPolygon[3].lon = -119.814214;
*/
	//Sets robot to "MANUAL" mode
	setBotMode( "MANUAL" , n );

	ros::spinOnce();
	r.sleep();

	//while ROS is still active
	while(ros::ok())
	{
	
		//randomly select a direction and time length
		randomDirection = rand() % 2;
		randomTime = getUniRand(1, 2);


		//spin backwards to choose a new, random direction
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

		while( boundaryCheck(quadPolygon) )
		{
			//return wheels to forward position
			setBotMovement( FORWARD_SPEED, STRAIGHT, rcPub );
			ros::spinOnce();
			r.sleep();
		}

		//head towards center until within boundary
		ROS_INFO("RETURNING TO BOUNDARY...");


		//force bot to move towards within polygon
		returnToBoundary( n , centerCoord );
		setBotMode( "AUTO" , n );

		while( !(boundaryCheck(quadPolygon)) )
		{
			ros::spinOnce();
			r.sleep();
		}
		setBotMode( "MANUAL" , n );
	

		//move forward for a small amount of time
		//ensures bot is completely within boundaries	
		stopTime = time(NULL) + 2;
		while( time(NULL) < stopTime )
		{
			setBotMovement( FORWARD_SPEED, STRAIGHT, rcPub );
			ros::spinOnce();
			r.sleep();
		}
		setBotMovement( ZERO_SPEED, STRAIGHT, rcPub );

		ros::spinOnce();
	}

	return 0;
}
