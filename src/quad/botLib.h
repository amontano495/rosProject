#ifndef BOTLIB_H
#define BOTLIB_H

#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/WaypointPush.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

struct coord {
	double lat;
	double lon;
} ;

//Sets the global coords to what the GPS device responds
void getBotCoords( const sensor_msgs::NavSatFix& msg );

//Determines if the bot is within the given two vertices
//The two vertices represent opposite corners of a given square
bool boundaryCheck( coord thePath[] );

void setBotMode( std::string mode, ros::NodeHandle &node );

bool RayCrossesSegment( coord a, coord b );

//Returns a uniformly distributed random number each time it is called
int getUniRand( int min, int max );

//pushes waypoints to the rover
bool waypointPusher( mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node, 
	int frame, int command, bool isCurrent, bool autoCont, 
	float param1, float param2, float param3, float param4, 
	float lat, float lon, float alt );

//Sends the rover back to within the boundaries
void returnToBoundary( ros::NodeHandle &node );

//Captures the vertices of the polygon
void setPolyVerts();

//Updates the speed of the bot
void setBotMovement( int speed, int angle, ros::Publisher &rcPub );


bool withinWaypointRadius( double lat, double lon );
#endif
