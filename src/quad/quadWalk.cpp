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

struct coord {
	double lat;
	double lon;
} ;

//Global gps coords of the robot
double botLat;
double botLon;
bool boundaryReached;

//Sets the current mode of the robot (MANUAL, AUTO, etc);
void setBotMode( std::string mode, ros::NodeHandle &node );

//Sets the global coords to what the GPS device responds
void getBotCoords( const sensor_msgs::NavSatFix& msg );

//Determines if the bot is within the given polygon
bool boundaryCheck( coord thePath[] );

bool RayCrossesSegment( coord a, coord b );

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
			if( !(boundaryCheck(quadPolygon)) || returning )
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

bool boundaryCheck( coord thePath[] )
{
	int crossings = 0;
	int count = 4;

	bool botWithinBoundary;

	int j;
	coord a, b;

	for( int i = 0; i < count; i++ )
	{
		a = thePath[i];
		j = i + 1;
		if( j >= count )
			j = 0;
		b = thePath[j];
		if( RayCrossesSegment( a, b ) )
			crossings++;
	}

	if( crossings % 2 == 1 )
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

bool RayCrossesSegment( coord a, coord b )
{
	double px = botLon;
	double py = botLat;
	double ax = a.lon;
	double ay = a.lat;
	double bx = b.lon;
	double by = b.lat;
	if( ay > by )
	{
		ax = b.lon;
		ay = b.lat;
		bx = a.lon;
		by = b.lat;
	}

	if (px < 0) { px += 360; };
	if (ax < 0) { ax += 360; };
	if (bx < 0) { bx += 360; };

	if (py == ay || py == by) 
		py += 0.00000001;
	if ((py > by || py < ay) || (px > fmax(ax,bx)))
		return false;
	if (px < fmin(ax,bx))
		return true;

	double red = (ax != bx) ? ((by - ay) / (bx - ax)) : FLT_MAX;
	double blue = (ax != px) ? ((py - ay) / (px - ax)) : FLT_MAX;
	return (blue >= red);
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

