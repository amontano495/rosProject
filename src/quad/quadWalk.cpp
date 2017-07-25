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

#define REVERSE_SPEED 1400
#define FORWARD_SPEED 1600
#define ZERO_SPEED 1500
#define LEFT_TURN 1100
#define RIGHT_TURN 1900
#define STRAIGHT 1500
#define LEFT 0
#define RIGHT 1

//Global gps coords of the robot
double botLat;
double botLon;
bool boundaryReached;

//Sets the current mode of the robot (MANUAL, AUTO, etc);
void setBotMode( std::string mode, ros::NodeHandle &node );

//Sets the global coords to what the GPS device responds
void getBotCoords( const sensor_msgs::NavSatFix& msg );

//Determines if the bot is within the given two vertices
//The two vertices represent opposite corners of a given square
bool boundaryCheck( double V1lat, double V1lon, double V2lat, double V2lon );

//Returns a uniformly distributed random number each time it is called
int getUniRand( int min, int max );

//Updates the speed of the bot
void setBotMovement( int speed, int angle, ros::Publisher &rcPub );

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
	bool returning;

	//Used to gain information from GPS device
	ros::Subscriber gpsSub = n.subscribe("/mavros/global_position/raw/fix", 1000, &getBotCoords );

	ros::Publisher rcPub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);


	//fixed time from user input
	int inputMoveTime = 20;

	//Represents two vertices of opposite corners of a square
	double boundaryV1lat;
	double boundaryV1lon;

	double boundaryV2lat;
	double boundaryV2lon;

	bool boundaryRecorded = false;
	bool northCornerRecorded = false;
	bool southCornerRecorded = false;
	char inputChar;

/*
	ROS_INFO("Please record the NORTH (N) and SOUTH (S) vertices of the boundaries");

	while( !boundaryRecorded )
	{
		std::cin >> inputChar;
	
		if( inputChar == 'S' )
		{
			ros::spinOnce();
			boundaryV1lat = botLat;
			boundaryV1lon = botLon;
			ROS_INFO_STREAM("SW CORNER LAT: " << std::setprecision(10) << boundaryV1lat << " LON: " << std::setprecision(10) << boundaryV1lon );
			southCornerRecorded = true;
		}
		else if( inputChar == 'N' )
		{
			ros::spinOnce();
			boundaryV2lat = botLat;
			boundaryV2lon = botLon;
			ROS_INFO_STREAM("NE CORNER LAT: " << std::setprecision(10) << boundaryV2lat << " LON: " << std::setprecision(10) << boundaryV2lon );
			northCornerRecorded = true;
		}

		if( northCornerRecorded && southCornerRecorded )
			boundaryRecorded = true;
	}
*/

	boundaryV2lat = 39.539065;
	boundaryV2lon = -119.8146213;

	boundaryV1lat = 39.5378701;
	boundaryV1lon = -119.8137527;

	//Sets robot to "MANUAL" mode
	setBotMode( "MANUAL" , n );

	while(ros::ok())
	{
		randomDirection = rand() % 2;
		randomTime = getUniRand(1, 2);

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

			//If out of bounds
			if( !(boundaryCheck(boundaryV1lat, boundaryV1lon, boundaryV2lat, boundaryV2lon)) || returning )
			{
				returning = true;
				ROS_INFO("HIT BOUNDARY");
				
				backTime = time(NULL) + 5;
				while( time(NULL) < backTime )
				{
					setBotMovement( REVERSE_SPEED, STRAIGHT, rcPub );
					ros::spinOnce();
					r.sleep();
				}
				returning = false;

				setBotMovement( ZERO_SPEED, STRAIGHT, rcPub );
				//setBotMovement( FORWARD_SPEED, STRAIGHT, rcPub );
				traveling = false;
			}

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

void setBotMovement( int speed, int angle, ros::Publisher &rcPub )
{
	mavros_msgs::OverrideRCIn msg;

	msg.channels[0] = angle;
	msg.channels[2] = speed;
	rcPub.publish(msg);
}

int getUniRand( int min, int max )
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(min, max);

	return dis(gen);
}

bool boundaryCheck( double V1lat, double V1lon, double V2lat, double V2lon )
{
	bool botWithinBoundary;

	if( (V1lat < botLat && botLat < V2lat) && (V1lon > botLon && botLon > V2lon) )
	{
		botWithinBoundary = true;
	}

	else
	{
		botWithinBoundary = false;
		ROS_WARN_STREAM("OUT OF BOUNDS! BOT LAT: " << std::setprecision(10) << botLat << " BOT LON: " << std::setprecision(10) << botLon );
	}

	return botWithinBoundary;
}

void getBotCoords( const sensor_msgs::NavSatFix &msg )
{
	botLat = msg.latitude;
	botLon = msg.longitude;
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

