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

struct coord {
	double lat;
	double lon;
} ;

//Global gps coords of the robot
double botLat;
double botLon;

//Global gps coords of the polygonal boundary
coord NWvert;
coord SWvert;
coord NEvert;
coord SEvert;
coord CenterVert;


//Sets the global coords to what the GPS device responds
void getBotCoords( const sensor_msgs::NavSatFix& msg );

//Determines if the bot is within the given two vertices
//The two vertices represent opposite corners of a given square
bool boundaryCheck( coord thePath[] );

bool RayCrossesSegment( coord a, coord b );

//Returns a uniformly distributed random number each time it is called
int getUniRand( int min, int max );

//Updates the speed of the bot
void setBotMovement( int speed, int angle, ros::Publisher &rcPub );

//pushes waypoints to the rover
bool waypointPusher( mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node, 
	int frame, int command, bool isCurrent, bool autoCont, 
	float param1, float param2, float param3, float param4, 
	float lat, float lon, float alt );

//Sends the rover back to within the boundaries
void returnToBoundary( ros::NodeHandle &node );

//Captures the vertices of the polygon
void setPolyVerts();

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

	//Lat/Lon vertices of the polygon
	//Must be in order

//	setPolyVerts();

	quadPolygon[0] = NWvert;
	quadPolygon[1] = SWvert;
	quadPolygon[2] = NEvert;
	quadPolygon[3] = SEvert;

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

	//Sets robot to "MANUAL" mode
	setBotMode( "MANUAL" , n );

	while( botLat < 1 )
	{
		ros::spinOnce();
		r.sleep();
	}

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

		while( boundaryCheck(quadPolygon) )
		{
			//return wheels to forward position
			setBotMovement( FORWARD_SPEED, STRAIGHT, rcPub );
			ros::spinOnce();
			r.sleep();
		}

		//head towards center until within boundary
		ROS_INFO("RETURNING TO BOUNDARY...");

		returnToBoundary( n );
		setBotMode( "AUTO" , n );

		while( !(boundaryCheck(quadPolygon)) )
		{
			ros::spinOnce();
			r.sleep();
		}
		setBotMode( "MANUAL" , n );
		
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


bool waypointPusher( mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node, 
	int frame, int command, bool isCurrent, bool autoCont, 
	float param1, float param2, float param3, float param4, 
	float lat, float lon, float alt )
{
	client = node.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	
	mavros_msgs::Waypoint nextWaypoint;

	nextWaypoint.frame = frame;
	nextWaypoint.command = command;
	nextWaypoint.is_current = isCurrent;
	nextWaypoint.autocontinue = autoCont;
	nextWaypoint.param1 = param1;
	nextWaypoint.param2 = param2;
	nextWaypoint.param3 = param3;
	nextWaypoint.param4 = param4;
	nextWaypoint.x_lat = lat;
	nextWaypoint.y_long = lon;
	nextWaypoint.z_alt = alt;

	pusher.request.waypoints.push_back(nextWaypoint);

	if( client.call( pusher) )
		ROS_INFO_STREAM("PUSHED WAYPOINT");
	else
		ROS_INFO_STREAM("PUSH FAILED");

	return client.call(pusher);
}

void returnToBoundary( ros::NodeHandle &node )
{
	ros::ServiceClient pushClient;
	mavros_msgs::WaypointPush wayPusher;

	waypointPusher( wayPusher, pushClient, node, 2, 22, true, true, 15, 0, 0, 0, 39.538440, -119.814151, 50 );
	waypointPusher( wayPusher, pushClient, node, 3, 16, true, true, 0, 0, 0, 0, 39.538440, -119.814151, 50 );
}

void setPolyVerts()
{
	int vertCounter = 0;
	std::string vertUserInput;

	while( vertCounter < 4 )
	{
		std::cin >> vertUserInput;
		ros::spinOnce();
		if( vertUserInput == "NW" )
		{
			NWvert.lat = botLat;
			NWvert.lon = botLon;
			vertCounter++;
		}
		else if( vertUserInput == "NE" )
		{
			NEvert.lat = botLat;
			NEvert.lon = botLon;
			vertCounter++;
		}
		else if( vertUserInput == "SW" )
		{
			SWvert.lat = botLat;
			SWvert.lon = botLon;
			vertCounter++;
		}
		else if( vertUserInput == "SE" )
		{
			SEvert.lat = botLat;
			SEvert.lon = botLon;
			vertCounter++;
		}
	}
}
